package l4cache

import chisel3._
import chisel3.util._
import chisel3.util.random._
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.amba.axi4._
import scala.math._
import utils.GTimer

case object NL2CacheCapacity extends Field[Int](2048)
case object NL2CacheWays extends Field[Int](16)

class L4MetadataEntry(tagBits: Int) extends Bundle {
  val valid = Bool()
  val dirty = Bool()
  val tag = UInt(width = tagBits.W)
  // override def cloneType = new L4MetadataEntry(tagBits).asInstanceOf[this.type]
}

// ============================== DCache ==============================
// TODO list:
// 1. powerful stats (show all counters [nHit, nAccess] for every 10K cycles)
// 2. [Done] integrate tag, vb, and db into MetaEntry
// 3. [Done] Support large block size (and validate)
// 4. Add sub-blocking into MetaEntry.

class AXI4SimpleL4Cache()(implicit p: Parameters) extends LazyModule
{
  val node = AXI4AdapterNode()

  lazy val module = new LazyModuleImp(this) {
    val nWays = p(NL4CacheWays)
    val nSets = p(NL4CacheCapacity) * 1024 / 64 / nWays
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      require(isPow2(nSets))
      require(isPow2(nWays))

      val Y = true.B
      val N = false.B
      val debug = false

      val blockSizeBits = 64 * 8
      val blockBytes = blockSizeBits / 8
      val innerBeatSize = in.r.bits.params.dataBits
      val innerBeatBytes = innerBeatSize / 8
      val innerDataBeats = blockSizeBits / innerBeatSize
      val innerBeatBits = log2Ceil(innerBeatBytes)
      val innerBeatIndexBits = log2Ceil(innerDataBeats)
      val innerBeatLSB = innerBeatBits
      val innerBeatMSB = innerBeatLSB + innerBeatIndexBits - 1

      val outerBeatSize = out.r.bits.params.dataBits
      val outerBeatBytes = outerBeatSize / 8
      val outerDataBeats = blockSizeBits / outerBeatSize
      val addrWidth = in.ar.bits.params.addrBits
      val innerIdWidth = in.ar.bits.params.idBits
      val outerIdWidth = out.ar.bits.params.idBits

      // to keep L1 miss & L2 hit penalty small, inner axi bus width should be as large as possible
      // on loongson and zedboard, outer axi bus width are usually 32bit
      // so we require that innerBeatSize to be multiples of outerBeatSize
      val split = innerBeatSize / outerBeatSize
      val splitBits = log2Ceil(split)
      require(isPow2(split))

      val indexBits = log2Ceil(nSets)
      val blockOffsetBits = log2Ceil(blockBytes)
      val tagBits = addrWidth - indexBits - blockOffsetBits
      val offsetLSB = 0
      val offsetMSB = blockOffsetBits - 1
      val indexLSB = offsetMSB + 1
      val indexMSB = indexLSB + indexBits - 1
      val tagLSB = indexMSB + 1
      val tagMSB = tagLSB + tagBits - 1

      val rst_cnt = RegInit(0.asUInt(log2Up(2 * nSets + 1).W))
      val rst = (rst_cnt < (2 * nSets).U) && !reset.asBool
      when (rst) { rst_cnt := rst_cnt + 1.U }

      val s_idle :: s_gather_write_data :: s_send_bresp :: s_update_meta :: s_tag_read :: s_merge_put_data :: s_data_read :: s_data_write :: s_wait_ram_awready :: s_do_ram_write :: s_wait_ram_bresp :: s_wait_ram_arready :: s_do_ram_read :: s_data_resp :: Nil = Enum(14)

      val state = RegInit(s_idle)
      // state transitions for each case
      // read hit: s_idle -> s_tag_read -> s_data_read -> s_data_resp -> s_idle
      // read miss no writeback : s_idle -> s_tag_read -> s_wait_ram_arready -> s_do_ram_read -> s_data_write -> s_update_meta -> s_idle
      // read miss writeback : s_idle -> s_tag_read -> s_data_read -> s_wait_ram_awready -> s_do_ram_write -> s_wait_ram_bresp
      //                              -> s_wait_ram_arready -> s_do_ram_read -> s_data_write -> s_update_meta -> s_idle
      // write hit: s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_data_read -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      // write miss no writeback : s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_wait_ram_arready -> s_do_ram_read
      //                                  -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      // write miss writeback : s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_data_read -> s_wait_ram_awready -> s_do_ram_write -> s_wait_ram_bresp
      //                               -> s_wait_ram_arready -> s_do_ram_read -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      val timer = GTimer()
      val log_prefix = "cycle: %d [L4Cache] state %x "
      def log_raw(prefix: String, fmt: String, tail: String, args: Bits*) = {
        if (debug) {
          printf(prefix + fmt + tail, args:_*)
        }
      }

      /** Single log */
      def log(fmt: String, args: Bits*) = log_raw(log_prefix, fmt, "\n", timer +: state +: args:_*)
      /** Log with line continued */
      def log_part(fmt: String, args: Bits*) = log_raw(log_prefix, fmt, "", timer +: state +: args:_*)
      /** Log with nothing added */
      def log_plain(fmt: String, args: Bits*) = log_raw("", fmt, "", args:_*)

      val in_ar = in.ar.bits
      val in_aw = in.aw.bits
      val in_r = in.r.bits
      val in_w = in.w.bits
      val in_b = in.b.bits

      val out_ar = out.ar.bits
      val out_aw = out.aw.bits
      val out_r = out.r.bits
      val out_w = out.w.bits
      val out_b = out.b.bits

      val addr = Reg(UInt(addrWidth.W))
      val id = Reg(UInt(innerIdWidth.W))
      val inner_end_beat = Reg(UInt(4.W))
      val ren = RegInit(N)
      val wen = RegInit(N)

      val gather_curr_beat = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
      val gather_last_beat = gather_curr_beat === inner_end_beat
      val merge_curr_beat = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
      val merge_last_beat = merge_curr_beat === inner_end_beat
      val resp_curr_beat = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
      val resp_last_beat = resp_curr_beat === inner_end_beat

      // state transitions:
      // s_idle: idle state
      // capture requests
      CheckOneHot(Seq(in.ar.fire(), in.aw.fire(), in.r.fire(), in.w.fire(), in.b.fire()))
      when (state === s_idle) {
        when (in.ar.fire()) {
          ren := Y
          wen := N
          addr := in_ar.addr
          id := in_ar.id

          val start_beat = in_ar.addr(innerBeatMSB, innerBeatLSB)
            gather_curr_beat := start_beat
          merge_curr_beat := start_beat
          resp_curr_beat := start_beat
          inner_end_beat := start_beat + in_ar.len

          state := s_tag_read
        } .elsewhen (in.aw.fire()) {
          ren := N
          wen := Y
          addr := in_aw.addr
          id := in_aw.id

          val start_beat = in_aw.addr(innerBeatMSB, innerBeatLSB)
          gather_curr_beat := start_beat
          merge_curr_beat := start_beat
          resp_curr_beat := start_beat
          inner_end_beat := start_beat + in_aw.len

          state := s_gather_write_data
        } .elsewhen (in.r.fire() || in.w.fire() || in.b.fire()) {
          assert(N, "Inner axi Unexpected handshake")
        }
      }
      in.ar.ready := state === s_idle && !rst
      // fox axi, ready can depend on valid
      // deal with do not let ar, aw fire at the same time
      in.aw.ready := state === s_idle && ! in.ar.valid && !rst

      // s_gather_write_data:
      // gather write data
      val put_data_buf = Reg(Vec(outerDataBeats, UInt(outerBeatSize.W)))
      val put_data_mask = RegInit(VecInit(Seq.fill(outerDataBeats) { 0.U(outerBeatBytes.W) }))
      in.w.ready := state === s_gather_write_data
      when (state === s_gather_write_data && in.w.fire()) {
        gather_curr_beat := gather_curr_beat + 1.U
        for (i <- 0 until split) {
          put_data_buf((gather_curr_beat << splitBits) + i.U) := in_w.data(outerBeatSize * (i + 1) - 1, outerBeatSize * i)
          put_data_mask((gather_curr_beat << splitBits) + i.U) := in_w.strb(outerBeatBytes * (i + 1) - 1, outerBeatBytes * i)
        }
        when (gather_last_beat) {
          state := s_send_bresp
        }
        bothHotOrNoneHot(gather_last_beat, in_w.last, "L2 gather beat error")
      }

      // s_send_bresp:
      // send bresp, end write transaction
      in.b.valid := state === s_send_bresp
      in_b.id := id
      in_b.resp := 0.asUInt(2.W)
      //in_b.user := 0.asUInt(5.W)

      when (state === s_send_bresp && in.b.fire()) {
        state := s_tag_read
      }

      // s_tag_read: inspecting meta data
      val (meta_array, omSRAM) = DescribedSRAM(
        name = "L2_meta_array",
        desc = "L2 cache metadata array",
        size = nSets,
        data = Vec(nWays, new L4MetadataEntry(tagBits))
      )

      val meta_raddr = Mux(in.ar.fire(), in_ar.addr, addr)
      val meta_array_wen = rst || state === s_update_meta
      val meta_ridx = meta_raddr(indexMSB, indexLSB)

      val meta_rdata = meta_array.read(meta_ridx, !meta_array_wen)

      val idx = addr(indexMSB, indexLSB)
      val tag = addr(tagMSB, tagLSB)

      def wayMap[T <: Data](f: Int => T) = VecInit((0 until nWays).map(f))

      val vb_rdata = wayMap((w: Int) => meta_rdata(w).valid).asUInt
      val db_rdata = wayMap((w: Int) => meta_rdata(w).dirty).asUInt
      val tag_rdata = wayMap((w: Int) => meta_rdata(w).tag)

      val tag_eq_way = wayMap((w: Int) => tag_rdata(w) === tag)
      val tag_match_way = wayMap((w: Int) => tag_eq_way(w) && vb_rdata(w)).asUInt
      val hit = tag_match_way.orR
      val read_hit = hit && ren
      val write_hit = hit && wen
      val read_miss = !hit && ren
      val write_miss = !hit && wen
      val hit_way = Wire(Bits())
      hit_way := 0.U
      (0 until nWays).foreach(i => when (tag_match_way(i)) { hit_way := i.U })

      // use random replacement
      val lfsr = LFSR(log2Ceil(nWays), state === s_update_meta && !hit)
      val repl_way_enable = (state === s_idle && in.ar.fire()) || (state === s_send_bresp && in.b.fire())
      val repl_way = RegEnable(next = if(nWays == 1) 0.U else lfsr(log2Ceil(nWays) - 1, 0),
        init = 0.U, enable = repl_way_enable)

      // valid and dirty
      val need_writeback = vb_rdata(repl_way) && db_rdata(repl_way)
      val writeback_tag = tag_rdata(repl_way)
      val writeback_addr = Cat(writeback_tag, Cat(idx, 0.U(blockOffsetBits.W)))

      val read_miss_writeback = read_miss && need_writeback
      val read_miss_no_writeback = read_miss && !need_writeback
      val write_miss_writeback = write_miss && need_writeback
      val write_miss_no_writeback = write_miss && !need_writeback

      val need_data_read = read_hit || write_hit || read_miss_writeback || write_miss_writeback

      when (state === s_tag_read) {
        log("req: isread %x iswrite %x addr %x idx %d tag %x hit %d hit_way %x repl_way %x needwb %x wbaddr %x",
            ren, wen,
            addr,
            idx,
            tag,
            hit, hit_way,
            repl_way,
            need_writeback, writeback_addr)
        // printf("[L4cache] time %d req: isread %x iswrite %x addr %x idx %d tag %x hit %d hit_way %x repl_way %x needwb %x wbaddr %x\n",
        //     GTimer(), ren, wen,
        //     addr,
        //     idx,
        //     tag,
        //     hit, hit_way,
        //     repl_way,
        //     need_writeback, writeback_addr)
        // show all tags, valid and dirty bits
        // log("time: %d [L4Cache] s1 tags: ", GTimer())
        // for (i <- 0 until nWays) {
        //   log("%x ", tag_rdata(i))
        // }
        // log("\n")
        // log("time: %d [L4Cache] s1 vb: %x db: %x\n", GTimer(), vb_rdata, db_rdata)

        // check for cross cache line bursts
        assert(inner_end_beat < innerDataBeats.U, "cross cache line bursts detected")

        when (read_hit || write_hit || read_miss_writeback || write_miss_writeback) {
          state := s_data_read
        } .elsewhen (read_miss_no_writeback || write_miss_no_writeback) {
          // no need to write back, directly refill data
          state := s_wait_ram_arready
        } .otherwise {
          assert(N, "Unexpected condition in s_tag_read")
        }
      }

      // ###############################################################
      // #                  data array read/write                      #
      // ###############################################################
      val data_read_way = Mux(read_hit || write_hit, hit_way, repl_way)
      // incase it overflows
      val data_read_cnt = RegInit(0.asUInt((log2Ceil(innerDataBeats) + 1).W))
      val data_read_valid = (state === s_tag_read && need_data_read) || (state === s_data_read && data_read_cnt =/= innerDataBeats.U)
      val data_read_idx = idx << log2Ceil(innerDataBeats) | data_read_cnt
      val dout = Wire(Vec(split, UInt(outerBeatSize.W)))

      val data_write_way = Mux(write_hit, hit_way, repl_way)
      val data_write_cnt = Wire(UInt())
      val data_write_valid = state === s_data_write
      val data_write_idx = idx << log2Ceil(innerDataBeats) | data_write_cnt
      val din = Wire(Vec(split, UInt(outerBeatSize.W)))

      val data_arrays = Seq.fill(split) {
        DescribedSRAM(
          name = "L2_data_array",
          desc = "L2 data array",
          size = nSets * innerDataBeats,
          data = Vec(nWays, UInt(width = outerBeatSize.W))
        )
      }
      for (((data_array, omSRAM), i) <- data_arrays zipWithIndex) {
        when (data_write_valid) {
          log("write data array: %d idx %d way %d data %x\n",
            i.U, data_write_idx, data_write_way, din(i))
          data_array.write(data_write_idx, VecInit(Seq.fill(nWays) { din(i) }), (0 until nWays).map(data_write_way === _.U))
        }
        dout(i) := data_array.read(data_read_idx, data_read_valid && !data_write_valid)(data_read_way)
        when (RegNext(data_read_valid, N)) {
          log("read data array: %d idx %d way %d data %x\n",
            i.U, RegNext(data_read_idx), RegNext(data_read_way), dout(i))
        }
      }

      // s_data_read
      // read miss or need write back
      val data_buf = Reg(Vec(outerDataBeats, UInt(outerBeatSize.W)))
      when (data_read_valid) {
        data_read_cnt := data_read_cnt + 1.U
      }
      when (state === s_data_read) {
        for (i <- 0 until split) {
          data_buf(((data_read_cnt - 1.U) << splitBits) + i.U) := dout(i)
        }
        when (data_read_cnt === innerDataBeats.U) {
          data_read_cnt := 0.U
          when (read_hit) {
            state := s_data_resp
          } .elsewhen (read_miss_writeback || write_miss_writeback) {
            state := s_wait_ram_awready
          } .elsewhen (write_hit) {
            state := s_merge_put_data
          } .otherwise {
            assert(N, "Unexpected condition in s_data_read")
          }
        }
      }

      // s_merge_put_data: merge data_buf and put_data_buf, and store the final result in data_buf
      // the old data(either read from data array or from refill) resides in data_buf
      // the new data(gathered from inner write) resides in put_data_buf
      def mergePutData(old_data: UInt, new_data: UInt, wmask: UInt): UInt = {
        val full_wmask = FillInterleaved(8, wmask)
          ((~full_wmask & old_data) | (full_wmask & new_data))
      }
      when (state === s_merge_put_data) {
        merge_curr_beat := merge_curr_beat + 1.U
        for (i <- 0 until split) {
          val idx = (merge_curr_beat << splitBits) + i.U
          data_buf(idx) := mergePutData(data_buf(idx), put_data_buf(idx), put_data_mask(idx))
        }
        when (merge_last_beat) {
          state := s_data_write
        }
      }

      // s_data_write
      val (write_cnt, write_done) = Counter(state === s_data_write, innerDataBeats)
      data_write_cnt := write_cnt
      for (i <- 0 until split) {
        val idx = (data_write_cnt << splitBits) + i.U
        din(i) := data_buf(idx)
      }
      when (state === s_data_write && write_done) {
        state := s_update_meta
      }

      // s_update_meta
      val update_way = Mux(write_hit, hit_way, repl_way)
      val update_metadata = Wire(Vec(nWays, new L4MetadataEntry(tagBits)))
      val rst_metadata = Wire(Vec(nWays, new L4MetadataEntry(tagBits)))

      for (i <- 0 until nWays) {
        val metadata = rst_metadata(i)
        metadata.valid := false.B
        metadata.dirty := false.B
        metadata.tag := 0.U
      }

      for (i <- 0 until nWays) {
        val metadata = update_metadata(i)
        val is_update_way = update_way === i.U
        when (is_update_way) {
          metadata.valid := true.B
          metadata.dirty := db_rdata(i) | wen
            // Mux(read_hit, db_rdata(i),
            // Mux(read_miss, false.B, true.B))
          metadata.tag := tag
        } .otherwise {
          metadata.valid := vb_rdata(i)
          metadata.dirty := db_rdata(i)
          metadata.tag := tag_rdata(i)
        }
      }

      val meta_array_widx = Mux(rst, rst_cnt, idx)
      val meta_array_wdata = Mux(rst, rst_metadata, update_metadata)

      when (meta_array_wen) {
        meta_array.write(meta_array_widx, meta_array_wdata)
        when (!hit && !rst) {
          assert(update_way === repl_way, "update must = repl way when cache miss")
          log("update_tag: idx %d tag %x valid %x dirty %x repl_way %d\n", idx, update_metadata(update_way).tag, update_metadata(update_way).valid, update_metadata(update_way).dirty, repl_way)
        }
      }

      when (state === s_update_meta) {
        // refill done
        when (ren) {
          state := s_data_resp
        } .otherwise {
          state := s_idle
        }
      }

      // outer axi interface
      // external memory bus width is 32/64/128bits
      // so each memport read/write is mapped into a whole axi bus width read/write
      val axi4_size = log2Up(outerBeatBytes).U
      val mem_addr = Cat(addr(tagMSB, indexLSB), 0.asUInt(blockOffsetBits.W))

      // #########################################################
      // #                  write back path                      #
      // #########################################################
      // s_wait_ram_awready
      when (state === s_wait_ram_awready && out.aw.fire()) {
        state := s_do_ram_write
      }

      val (wb_cnt, wb_done) = Counter(out.w.fire() && state === s_do_ram_write, outerDataBeats)
      when (state === s_do_ram_write && wb_done) {
        state := s_wait_ram_bresp
      }
      // when (!reset.asBool && out.w.fire()) {
        bothHotOrNoneHot(wb_done, out_w.last, "L2 write back error")
      // }

      // write address channel signals
      out_aw.id := 0.asUInt(outerIdWidth.W)
      out_aw.addr := writeback_addr
      out_aw.len := (outerDataBeats - 1).asUInt(8.W)
      out_aw.size := axi4_size
      out_aw.burst := "b01".U       // normal sequential memory
      out_aw.lock := 0.asUInt(1.W)
      out_aw.cache := "b1111".U
      out_aw.prot := 0.asUInt(3.W)
      //out_aw.region := 0.asUInt(4.W)
      out_aw.qos := 0.asUInt(4.W)
      //out_aw.user := 0.asUInt(5.W)
      out.aw.valid := state === s_wait_ram_awready

      // write data channel signals
      //out_w.id := 0.asUInt(outerIdWidth.W)
      out_w.data := data_buf(wb_cnt)
      out_w.strb := Fill(outerBeatBytes, 1.asUInt(1.W))
      out_w.last := wb_cnt === (outerDataBeats - 1).U
      //out_w.user := 0.asUInt(5.W)
      out.w.valid := state === s_do_ram_write

      when (state === s_wait_ram_bresp && out.b.fire()) {
        when (read_miss_writeback || write_miss_writeback) {
          // do refill
          state := s_wait_ram_arready
        } .otherwise {
          assert(N, "Unexpected condition in s_wait_ram_bresp")
        }
      }

      // write response channel signals
      out.b.ready := state === s_wait_ram_bresp

      // #####################################################
      // #                  refill path                      #
      // #####################################################
      when (state === s_wait_ram_arready && out.ar.fire()) {
        state := s_do_ram_read
      }
      val (refill_cnt, refill_done) = Counter(out.r.fire() && state === s_do_ram_read, outerDataBeats)
      when (state === s_do_ram_read && out.r.fire()) {
        data_buf(refill_cnt) := out_r.data
        when (refill_done) {
          when (ren) {
            state := s_data_write
          } .otherwise {
            state := s_merge_put_data
          }
        }
        bothHotOrNoneHot(refill_done, out_r.last, "L2 refill error")
      }

      // read address channel signals
      out_ar.id := 0.asUInt(outerIdWidth.W)
      out_ar.addr := mem_addr
      out_ar.len := (outerDataBeats - 1).asUInt(8.W)
      out_ar.size := axi4_size
      out_ar.burst := "b01".U // normal sequential memory
      out_ar.lock := 0.asUInt(1.W)
      out_ar.cache := "b1111".U
      out_ar.prot := 0.asUInt(3.W)
      //out_ar.region := 0.asUInt(4.W)
      out_ar.qos := 0.asUInt(4.W)
      //out_ar.user := 0.asUInt(5.W)
      out.ar.valid := state === s_wait_ram_arready

      // read data channel signals
      out.r.ready := state === s_do_ram_read


      // ########################################################
      // #                  data resp path                      #
      // ########################################################
      when (state === s_data_resp && in.r.fire()) {
        resp_curr_beat := resp_curr_beat + 1.U
        when (resp_last_beat) {
          state := s_idle
        }
      }

      val resp_data = Wire(Vec(split, UInt(outerBeatSize.W)))
      for (i <- 0 until split) {
        resp_data(i) := data_buf((resp_curr_beat << splitBits) + i.U)
      }
      in_r.id := id
      in_r.data := resp_data.asUInt
      in_r.resp := 0.asUInt(2.W)
      in_r.last := resp_last_beat
      //in_r.user := 0.asUInt(5.W)
      in.r.valid := state === s_data_resp

      // Stat counters
      L4PerfAccumulator("l4_req", state === s_tag_read) // tag_read lasts one cycle per request
      L4PerfAccumulator("l4_req_hit", state === s_tag_read && hit)
    }
  }
}

object AXI4SimpleL4Cache
{
  def apply()(implicit p: Parameters): AXI4Node =
  {
    if (p(NL4CacheCapacity) != 0) {
      val axi4simpleL4cache = LazyModule(new AXI4SimpleL4Cache())
      axi4simpleL4cache.node
    }
    else {
      val axi4simpleL4cache = LazyModule(new AXI4Buffer(BufferParams.none))
      axi4simpleL4cache.node
    }
  }
}

object AXI4SimpleL4CacheRef
{
  def apply()(implicit p: Parameters): AXI4SimpleL4Cache = {
    val axi4simpleL4cache = LazyModule(new AXI4SimpleL4Cache())
    axi4simpleL4cache
  }
}
