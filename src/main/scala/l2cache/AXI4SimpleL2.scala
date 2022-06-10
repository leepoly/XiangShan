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

case object NL4CacheCapacity extends Field[Int](8192)
case object NL4CacheWays extends Field[Int](16)
case object NL4BanksPerMemChannel extends Field[Int](4)

class L4MetadataEntry(tagBits: Int, nSubblk: Int) extends Bundle {
  val valid = Bool()
  val dirty = Bool() // TODO: remove me
  val tag = UInt(width = tagBits.W)
  val subblockValid = UInt(width = nSubblk.W)
  val subblockDirty = UInt(width = nSubblk.W)
  // override def cloneType = new L4MetadataEntry(tagBits).asInstanceOf[this.type]
}

// ============================== DCache ==============================
// TODO list:
// 1. powerful stats (show all counters [nHit, nAccess] for every 10K cycles)
// 2. [Done] integrate tag, vb, and db into MetaEntry
// 3. [Done] Support large block size (and validate)
// 4. [Done] Add sub-blocking into MetaEntry.

class AXI4SimpleL4Cache()(implicit p: Parameters) extends LazyModule
{
  val node = AXI4AdapterNode()

  lazy val module = new LazyModuleImp(this) {
    val subblockSize = 64 // in bytes
    val nSubblk = 2
    val blockSize = subblockSize * nSubblk
    val nWays = p(NL4CacheWays)
    val nSets = p(NL4CacheCapacity) / blockSize / nWays
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      require(isPow2(nSets))
      require(isPow2(nWays))
      require(isPow2(subblockSize))
      require(isPow2(nSubblk))

      val Y = true.B
      val N = false.B
      val debug = false

      val subblockSizeBits = subblockSize * 8
      val subblockBytes = subblockSize
      val blockSizeBits = blockSize * 8
      val innerBeatSize = in.r.bits.params.dataBits
      val innerBeatBytes = innerBeatSize / 8
      val innerDataBeats = subblockSizeBits / innerBeatSize
      val innerBlockBeats = blockSizeBits / innerBeatSize
      val innerBeatBits = log2Ceil(innerBeatBytes)
      val innerBeatIndexBits = log2Ceil(innerDataBeats)
      val innerBeatLSB = innerBeatBits
      val innerBeatMSB = innerBeatLSB + innerBeatIndexBits - 1

      val outerBeatSize = out.r.bits.params.dataBits
      val outerBeatBytes = outerBeatSize / 8
      val outerSubblkBeats = subblockSizeBits / outerBeatSize
      val addrWidth = in.ar.bits.params.addrBits
      val innerIdWidth = in.ar.bits.params.idBits
      val outerIdWidth = out.ar.bits.params.idBits

      // to keep L1 miss & L2 hit penalty small, inner axi bus width should be as large as possible
      // on loongson and zedboard, outer axi bus width are usually 32bit
      // so we require that innerBeatSize to be multiples of outerBeatSize
      val split = innerBeatSize / outerBeatSize
      val splitBits = log2Ceil(split)
      require(isPow2(split))
      assert(split == 1) // TODO: I guess split is no use
      println("innerBeatBytes %d innerDataBeats %d", innerBeatBytes, innerDataBeats)
      println("outerSubblkBeats %d", outerSubblkBeats)
      assert(innerBeatBytes == 32)
      assert(innerDataBeats == 2)
      assert(outerBeatBytes == 32)
      assert(outerSubblkBeats == 2)

      val indexBits = log2Ceil(nSets)
      val subblockOffsetBits = log2Ceil(subblockBytes)
      val subblkIdBits = log2Ceil(nSubblk)
      assert(subblkIdBits > 0)
      val tagBits = addrWidth - indexBits - subblkIdBits - subblockOffsetBits
      val offsetLSB = 0
      val offsetMSB = subblockOffsetBits - 1
      val subblkIdLSB = offsetMSB + 1
      val subblkIdMSB = subblkIdLSB + subblkIdBits - 1
      val indexLSB = subblkIdMSB + 1
      val indexMSB = indexLSB + indexBits - 1
      val tagLSB = indexMSB + 1
      val tagMSB = tagLSB + tagBits - 1
      assert(tagMSB + 1 == addrWidth)

      val rst_cnt = RegInit(0.U(log2Up(2 * nSets + 1).W))
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

      val gather_curr_beat = RegInit(0.U(innerBeatIndexBits.W))
      val gather_last_beat = gather_curr_beat === inner_end_beat
      val merge_curr_beat = RegInit(0.U(innerBeatIndexBits.W))
      val merge_last_beat = merge_curr_beat === inner_end_beat
      val resp_curr_beat = RegInit(0.U(innerBeatIndexBits.W))
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
      val put_data_buf = Reg(Vec(outerSubblkBeats, UInt(outerBeatSize.W))) // lineSize
      val put_data_mask = RegInit(VecInit(Seq.fill(outerSubblkBeats) { 0.U(outerBeatBytes.W) })) // lineSize
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
        bothHotOrNoneHot(gather_last_beat, in_w.last, "L4 gather beat error")
      }

      // s_send_bresp:
      // send bresp, end write transaction
      in.b.valid := state === s_send_bresp
      in_b.id := id
      in_b.resp := 0.U(2.W)
      //in_b.user := 0.U(5.W)

      when (state === s_send_bresp && in.b.fire()) {
        state := s_tag_read
      }

      // s_tag_read: inspecting meta data
      val (meta_array, omSRAM) = DescribedSRAM(
        name = "L4_meta_array",
        desc = "L4 cache metadata array",
        size = nSets,
        data = Vec(nWays, new L4MetadataEntry(tagBits, nSubblk))
      )

      val meta_raddr = Mux(in.ar.fire(), in_ar.addr, addr)
      val meta_array_wen = rst || state === s_update_meta
      val meta_ridx = meta_raddr(indexMSB, indexLSB)

      val meta_rdata = meta_array.read(meta_ridx, !meta_array_wen)

      val subblkId = addr(subblkIdMSB, subblkIdLSB)
      val idx = addr(indexMSB, indexLSB)
      val tag = addr(tagMSB, tagLSB)

      def wayMap[T <: Data](f: Int => T) = VecInit((0 until nWays).map(f))

      val block_vb_rdata = wayMap((w: Int) => meta_rdata(w).valid).asUInt
      val block_db_rdata = wayMap((w: Int) => meta_rdata(w).dirty).asUInt // TODO: remove me
      val tag_rdata = wayMap((w: Int) => meta_rdata(w).tag)
      val subblock_vb_rdata = wayMap((w: Int) => meta_rdata(w).subblockValid)
      val subblock_db_rdata = wayMap((w: Int) => meta_rdata(w).subblockDirty)

      val tag_eq_way = wayMap((w: Int) => tag_rdata(w) === tag)
      val tag_match_way = wayMap((w: Int) => tag_eq_way(w) && block_vb_rdata(w)).asUInt
      val hit_way = Wire(Bits())
      hit_way := 0.U
      (0 until nWays).foreach(i => when (tag_match_way(i)) { hit_way := i.U })
      val block_hit = tag_match_way.orR // block hit/miss
      val block_read_hit = block_hit && ren
      val block_write_hit = block_hit && wen
      val block_read_miss = !block_hit && ren
      val block_write_miss = !block_hit && wen
      val subblock_hit = block_hit && subblock_vb_rdata(hit_way)(subblkId) // subblk hit/miss
      val subblock_read_hit = subblock_hit && ren // situation 1
      val subblock_write_hit = subblock_hit && wen // situation 2
      val onlysubblock_read_miss = block_hit && !subblock_hit && ren // situation 3: block hit, but subblk miss
      val onlysubblock_write_miss = block_hit && !subblock_hit && wen // situation 4

      // use random replacement
      val lfsr = LFSR(log2Ceil(nWays), state === s_update_meta && !block_hit)
      val repl_way_enable = (state === s_idle && in.ar.fire()) || (state === s_send_bresp && in.b.fire())
      val repl_way = RegEnable(next = if(nWays == 1) 0.U else lfsr(log2Ceil(nWays) - 1, 0),
        init = 0.U, enable = repl_way_enable)
      // val repl_way = 0.U

      // valid and dirty
      // writeback in the block level
      val need_writeback = !block_hit && block_vb_rdata(repl_way) && block_db_rdata(repl_way)
      // val need_writeback = !block_hit && true.B
      // TODO: need_writeback: sub-block level
      val writeback_tag = tag_rdata(repl_way)
      val wb_subblkId = RegInit(0.U((subblkIdBits).W))
      val writeback_addr = Cat(writeback_tag, Cat(idx, Cat(wb_subblkId, 0.U(subblockOffsetBits.W))))

      // TODO: adjust accordingly
      val block_read_miss_writeback = block_read_miss && need_writeback // situation 5
      val block_read_miss_no_writeback = block_read_miss && !need_writeback // situation 6
      val block_write_miss_writeback = block_write_miss && need_writeback // situation 7
      val block_write_miss_no_writeback = block_write_miss && !need_writeback // situation 8

      val need_data_read = subblock_read_hit || subblock_write_hit || // subblk hit
                           block_read_miss_writeback || block_write_miss_writeback // block miss, need writeback
                            // situation 1, 2, 5, 7
      val no_need_data_read = onlysubblock_read_miss || onlysubblock_write_miss ||
                              block_read_miss_no_writeback || block_write_miss_no_writeback
                            // situation: 3, 4, 6, 8

      when (state === s_tag_read) {
        log("req: isread %x iswrite %x addr %x idx %d tag %x subblkId %x blkHit %d subblkHit %d hit_way %x",
            ren, wen,
            addr,
            idx, tag, subblkId,
            block_hit, subblock_hit, hit_way)
        when (!block_hit) {
          log("\tmiss repl_way %x repl_valid %x repl_dirty %x repl_subblkValid %x repl_subblkDirty %x needwb %x wbaddr %x",
              repl_way, block_vb_rdata(repl_way), block_db_rdata(repl_way), subblock_vb_rdata(repl_way), subblock_db_rdata(repl_way), need_writeback, writeback_addr)
        } .otherwise {
          log("\thit hit_way %x hit_valid %x hit_dirty %x hit_subblkValid %x hit_subblkDirty %x",
              hit_way, block_vb_rdata(hit_way), block_db_rdata(hit_way), subblock_vb_rdata(hit_way), subblock_db_rdata(hit_way))
        }
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
        // log("time: %d [L4Cache] s1 vb: %x db: %x\n", GTimer(), block_vb_rdata, block_db_rdata)

        // check for cross cache line bursts
        assert(inner_end_beat < innerDataBeats.U, "cross cache line bursts detected")

        when (need_data_read) {
          state := s_data_read
          when (need_writeback) {
            log("writeback start: idx %d way %x addr %x subblkId %x ", idx, repl_way, writeback_addr, wb_subblkId)
          }
        } .elsewhen (no_need_data_read) {
          // no need to write back, directly refill data
          state := s_wait_ram_arready
        } .otherwise {
          assert(N, "Unexpected condition in s_tag_read")
        }
      }

      // ###############################################################
      // #                  data array read/write                      #
      // ###############################################################
      // data array read idx and beat number are different for situation (5, 7) and (1, 2)
      val data_read_way = Mux(block_hit, hit_way, repl_way)
      // incase it overflows
      val data_read_cnt = RegInit(0.U((innerBeatIndexBits + 1).W))
      val data_read_is_last_beat = data_read_cnt === innerDataBeats.U
      val data_read_valid = (state === s_tag_read && need_data_read) || (state === s_data_read && !data_read_is_last_beat)
      val data_read_idx = Mux(need_writeback, (idx << (innerBeatIndexBits + subblkIdBits)) | (wb_subblkId << innerBeatIndexBits) | data_read_cnt,
                              (idx << (innerBeatIndexBits + subblkIdBits)) | (subblkId << innerBeatIndexBits) | data_read_cnt)
      val dout = Wire(Vec(split, UInt(outerBeatSize.W)))

      val data_write_way = Mux(block_hit, hit_way, repl_way)
      val data_write_cnt = Wire(UInt())
      val data_write_valid = state === s_data_write
      val data_write_idx = (idx << (innerBeatIndexBits + subblkIdBits)) | (subblkId << innerBeatIndexBits) | data_write_cnt
      val din = Wire(Vec(split, UInt(outerBeatSize.W)))

      val data_arrays = Seq.fill(split) {
        DescribedSRAM(
          name = "L4_data_array",
          desc = "L4 data array",
          size = nSets * nSubblk * innerDataBeats,
          data = Vec(nWays, UInt(width = outerBeatSize.W))
        )
      }
      for (((data_array, omSRAM), i) <- data_arrays zipWithIndex) {
        when (data_write_valid) {
          log("write data array: %d idx %d subblkId %d way %d cnt %d data %x\n",
            i.U, idx, subblkId, data_write_way, data_write_cnt, din(i))
          data_array.write(data_write_idx, VecInit(Seq.fill(nWays) { din(i) }), (0 until nWays).map(data_write_way === _.asUInt))
        }
        dout(i) := data_array.read(data_read_idx, data_read_valid && !data_write_valid)(data_read_way)
        when (RegNext(data_read_valid, N)) {
          val sel_subblkId = Mux(need_writeback, wb_subblkId, subblkId)
          log("read data array: %d idx %d subblkId %d way %d wb %x cnt %d data %x\n",
            i.U, RegNext(idx), RegNext(sel_subblkId), RegNext(data_read_way), RegNext(need_writeback), RegNext(data_read_cnt), dout(i))
        }
      }

      // s_data_read
      // read miss or need write back
      val data_buf = Reg(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
      when (data_read_valid) {
        data_read_cnt := data_read_cnt + 1.U
      }
      when (state === s_data_read) {
        for (i <- 0 until split) {
          data_buf(((data_read_cnt - 1.U) << splitBits) + i.U) := dout(i)
        }
        when (data_read_is_last_beat) {
          data_read_cnt := 0.U
          when (subblock_read_hit) {
            state := s_data_resp
          } .elsewhen (block_read_miss_writeback || block_write_miss_writeback) {
            state := s_wait_ram_awready
          } .elsewhen (subblock_write_hit) {
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
      val update_way = Mux(block_hit, hit_way, repl_way)
      val update_metadata = Wire(Vec(nWays, new L4MetadataEntry(tagBits, nSubblk)))
      val rst_metadata = Wire(Vec(nWays, new L4MetadataEntry(tagBits, nSubblk)))

      for (i <- 0 until nWays) {
        val metadata = rst_metadata(i)
        metadata.valid := false.B
        metadata.dirty := false.B
        metadata.tag := 0.U
        metadata.subblockValid := 0.U(nSubblk.W)
        metadata.subblockDirty := 0.U(nSubblk.W)
      }

      for (i <- 0 until nWays) {
        val metadata = update_metadata(i)
        val is_update_way = update_way === i.U
        when (is_update_way) {
          metadata.valid := true.B
          metadata.tag := tag
          metadata.subblockValid := Mux(block_hit, subblock_vb_rdata(i) | (1.U << subblkId),
                                    1.U << subblkId)
          metadata.subblockDirty := Mux(block_hit, subblock_db_rdata(i) | (wen << subblkId),
                                    (wen << subblkId))
          metadata.dirty := metadata.subblockDirty.orR
        } .otherwise {
          metadata.valid := block_vb_rdata(i)
          metadata.dirty := block_db_rdata(i)
          metadata.tag := tag_rdata(i)
          metadata.subblockValid := subblock_vb_rdata(i)
          metadata.subblockDirty := subblock_db_rdata(i)
        }
      }

      val meta_array_widx = Mux(rst, rst_cnt, idx)
      val meta_array_wdata = Mux(rst, rst_metadata, update_metadata)

      when (meta_array_wen) {
        meta_array.write(meta_array_widx, meta_array_wdata)
        when (!block_hit && !rst) {
          assert(update_way === repl_way, "update must = repl way when cache miss")
          log("update_tag: idx %d tag %x valid %x dirty %x subValid %x subDirty %x repl_way %d\n", idx, update_metadata(update_way).tag, update_metadata(update_way).valid, update_metadata(update_way).dirty, update_metadata(update_way).subblockValid, update_metadata(update_way).subblockDirty, repl_way)
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
      val mem_addr = Cat(addr(tagMSB, subblkIdLSB), 0.asUInt(subblockOffsetBits.W))

      // #########################################################
      // #                  write back path                      #
      // #########################################################
      // s_wait_ram_awready
      when (state === s_wait_ram_awready && out.aw.fire()) {
        state := s_do_ram_write
      }

      val (wb_cnt, wb_done) = Counter(out.w.fire() && state === s_do_ram_write, outerSubblkBeats)
      when (state === s_do_ram_write && wb_done) {
        state := s_wait_ram_bresp
      }
      when (state === s_do_ram_write && out.w.fire()) {
        log("data_writeback: idx %d tag %x mem_addr %x cnt %x wb_done %x wbSubId %x subValid %x subDirty %x wb_strb %x wb_data %x\n", idx, writeback_tag, writeback_addr, wb_cnt, wb_done, wb_subblkId, subblock_vb_rdata(repl_way), subblock_db_rdata(repl_way), out_w.strb, out_w.data)
      }
      when (!reset.asBool && out.w.fire()) {
        bothHotOrNoneHot(wb_done, out_w.last, "L4 write back error")
      }

      // write address channel signals
      out_aw.id := 0.U(outerIdWidth.W)
      out_aw.addr := writeback_addr
      out_aw.len := (outerSubblkBeats - 1).U(8.W)
      out_aw.size := axi4_size
      out_aw.burst := "b01".U       // normal sequential memory
      out_aw.lock := 0.U(1.W)
      out_aw.cache := "b1111".U
      out_aw.prot := 0.U(3.W)
      //out_aw.region := 0.U(4.W)
      out_aw.qos := 0.U(4.W)
      //out_aw.user := 0.U(5.W)
      out.aw.valid := state === s_wait_ram_awready

      // write data channel signals
      //out_w.id := 0.U(outerIdWidth.W)
      out_w.data := data_buf(wb_cnt)
      val out_w_strb = Mux(subblock_db_rdata(repl_way)(wb_subblkId), Fill(outerBeatBytes, 1.U(1.W)),
                            Fill(outerBeatBytes, 0.U(1.W))) // no-op for invalid lines
      out_w.strb := out_w_strb
      out_w.last := wb_cnt === (outerSubblkBeats - 1).U
      //out_w.user := 0.U(5.W)
      out.w.valid := state === s_do_ram_write

      when (state === s_wait_ram_bresp && out.b.fire()) {
        when (block_read_miss_writeback || block_write_miss_writeback) {
          when (wb_subblkId === (nSubblk - 1).U) {
            // do refill
            state := s_wait_ram_arready
            wb_subblkId := 0.U
            log("writeback complete: idx %d way %x addr %x subblkId %d ", idx, repl_way, writeback_addr, wb_subblkId)
          } .otherwise {
            log("writeback done for idx %d way %x addr %x subblkId %d ", idx, repl_way, writeback_addr, wb_subblkId)
            state := s_data_read
            wb_subblkId := wb_subblkId + 1.U
            assert(wb_subblkId <= nSubblk.U)
          }
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
      val (refill_cnt, refill_done) = Counter(out.r.fire() && state === s_do_ram_read, outerSubblkBeats)
      when (state === s_do_ram_read && out.r.fire()) {
        data_buf(refill_cnt) := out_r.data
        when (refill_done) {
          when (ren) {
            state := s_data_write
          } .otherwise {
            state := s_merge_put_data
          }
        }
        log("data_refill: idx %d tag %x subblkId %x mem_addr %x cnt %x refill_data %x\n", idx, tag, subblkId, mem_addr, refill_cnt, out_r.data)
        bothHotOrNoneHot(refill_done, out_r.last, "L4 refill error")
      }

      // read address channel signals
      out_ar.id := 0.U(outerIdWidth.W)
      out_ar.addr := mem_addr
      out_ar.len := (outerSubblkBeats - 1).U(8.W)
      out_ar.size := axi4_size
      out_ar.burst := "b01".U // normal sequential memory
      out_ar.lock := 0.U(1.W)
      out_ar.cache := "b1111".U
      out_ar.prot := 0.U(3.W)
      //out_ar.region := 0.U(4.W)
      out_ar.qos := 0.U(4.W)
      //out_ar.user := 0.U(5.W)
      out.ar.valid := state === s_wait_ram_arready

      // read data channel signals
      out.r.ready := state === s_do_ram_read


      // ########################################################
      // #                  data resp path                      #
      // ########################################################
      val resp_data = Wire(Vec(split, UInt(outerBeatSize.W)))
      for (i <- 0 until split) {
        resp_data(i) := data_buf((resp_curr_beat << splitBits) + i.U)
      }

      when (state === s_data_resp && in.r.fire()) {
        resp_curr_beat := resp_curr_beat + 1.U
        when (resp_last_beat) {
          state := s_idle
        }
        log("data_resp: idx %d tag %x subblkId %x req_addr %x cnt %x resp_data %x\n", idx, tag, subblkId, addr, resp_curr_beat, resp_data.asUInt)
      }

      in_r.id := id
      in_r.data := resp_data.asUInt
      in_r.resp := 0.U(2.W)
      in_r.last := resp_last_beat
      //in_r.user := 0.U(5.W)
      in.r.valid := state === s_data_resp

      // Stat counters
      L4PerfAccumulator("l4_req", state === s_tag_read) // tag_read lasts one cycle per request
      L4PerfAccumulator("l4_req_hit", state === s_tag_read && subblock_hit)
      L4PerfAccumulator("l4_req_block_hit", state === s_tag_read && block_hit)
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
