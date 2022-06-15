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

case object NL4CacheCapacity extends Field[Int](4096)
case object NL4CacheWays extends Field[Int](16)
case object NL4BanksPerMemChannel extends Field[Int](4)

class L4MetadataEntry(superblockTagBits: Int, nSubblk: Int, intraSlotIdBits: Int, intraSlotLenBits: Int) extends Bundle {
  val valid = Bool()
  val superblockTag = UInt(width = superblockTagBits.W)
  val subblockValid = UInt(width = nSubblk.W)
  val subblockDirty = UInt(width = nSubblk.W)
  val intraSlotId = Vec(nSubblk, UInt(intraSlotIdBits.W))
  val intraSlotLen = Vec(nSubblk, UInt(intraSlotLenBits.W))
}

class ZeroLeadingCompressor(outerSubblkBeats: Int, outerBeatSize: Int, intraSlotLenBits: Int) extends Module
{
  val io = IO(new Bundle() {
    val data = Input(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
    val data_lastbeat = Input(UInt(outerBeatSize.W)) // last beat data not stored to put_data Reg yet
    val first_line = Input(Bool())
    val last_line = Input(Bool())
    val line_ready = Input(Bool())
    val comp_len = Output(Bool())
    val comp_data = Output(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
  })

  val input_data = Wire(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
  (0 until outerSubblkBeats - 1).map((w: Int) => input_data(w) := io.data(w))
  input_data(outerSubblkBeats - 1) := io.data_lastbeat

  val comp_cnt = RegInit(1.U(intraSlotLenBits.W))
  val data_beat_or = VecInit((0 until outerSubblkBeats).map((w: Int) => input_data(w).orR))
  val data_or = RegInit(false.B)
  val first_line_data = Reg(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
  when (io.line_ready) {
    // data_or := true.B // TODO: always no compress
    data_or := data_or | data_beat_or.orR
    when (!(data_or | data_beat_or.orR)) {
      comp_cnt := comp_cnt + 1.U
    }
    when (io.first_line) {
      // first_line_data := input_data
      (0 until outerSubblkBeats).map((w: Int) => first_line_data(w) := input_data(w))
    }
    when (RegNext(comp_cnt) > 1.U) {
      printf("[Compressor] first_line %x data_beat_or %x data_or %x comp_cnt %x\n", io.first_line, data_beat_or.asUInt, data_or | data_beat_or.orR, RegNext(comp_cnt))
    }
    when (io.last_line) {
      comp_cnt := 1.U
      data_or := false.B
    }
    // (0 until outerSubblkBeats).map((w: Int) => {
    // printf("[Compressor] comp_data cnt %x input_data %x first_line_data %x\n", w.U, input_data(w), first_line_data(w))
    // })
  }
  io.comp_len := comp_cnt
  io.comp_data := first_line_data // if compressible, output first_line_data (all zero), otherwise output first_line_data
}

class ZeroLeadingDecompressor(outerSubblkBeats: Int, outerBeatSize: Int, intraSlotLenBits: Int) extends Module
{
  val io = IO(new Bundle() {
    val comp_data = Input(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
    val offset = Input(UInt(intraSlotLenBits.W))
    val comp_slotLen = Input(UInt(intraSlotLenBits.W))
    val ready = Input(Bool())
    val decomp_data = Output(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))
  })

  val empty_data = VecInit(Seq.fill(outerSubblkBeats) { 0.U(outerBeatSize.W) })
  io.decomp_data := Mux(io.comp_slotLen === 1.U, io.comp_data, empty_data)
}

// ============================== DCache ==============================
// TODO list:
// 1. [Done] powerful stats (show all counters [nHit, nAccess] for every 10K cycles)
// 2. [Done] integrate tag, vb, and db into MetaEntry
// 3. [Done] Support large block size (and validate)
// 4. [Done] Add sub-blocking into MetaEntry. (Level-1 style)
// 5. [Done] Add per-block dirty information.
// 6. [Done] Level-2 style sub-blocking
// 7. [Done] Level-3 Per-superblock tag and sub-blocking
// 8. [Done] Zero-leading compression (compress when superblock miss)

// Yage
// 1. Xiangshan environment setup (Docker, https://xiangshan-doc.readthedocs.io/zh_CN/latest/tools/xsenv/)
// 2. Read both cache implementation (This one, and the original cache from https://github.com/tsinghua-ideal/hybridmem-fpga/commit/60452d0a9f6651bdb089312fcd90afb845b1826d)
// 3. Low hit rate investigation (larger capacity? enable all subblock slots and all associativities? larger block size?)
// 4. More stat counters, e.g., average_compSlotLen (indicate the amount of saving space), num_hitFromSubblocking
// 5. More software: coremark, microbench, etc. [Optional] Try linux with arbitrary programs, Linux with stdin, etc. (Ref: https://github.dev/OpenXiangShan/riscv-linux)
// 6. Show verilog and additional synthesizing cost in vivado
// 7. At least one of the FPC/BDI compression (Frequent Pattern Compression: https://minds.wisconsin.edu/handle/1793/60388, Base-Delta-Immediate Compression: https://users.ece.cmu.edu/~omutlu/pub/bdi-compression_pact12.pdf)
// 8. At least one replacement policy: FIFO/CLOCK is fine
// 9. Insert more blank cycles to emulate fast memory latency (Ref)
// 10. More powerful compression: compress when sub-block miss, read/write hit, etc.
//      (bug) cycle 2920381 comp_slotLen==0
// 11. [Optional] Baryon's succient metadata format

class AXI4SimpleL4Cache()(implicit p: Parameters) extends LazyModule
{
  val node = AXI4AdapterNode()

  lazy val module = new LazyModuleImp(this) {
    val subblockSize = 64 // in bytes
    val nSubblk = 2
    val nBlockPerSuperblock = 2
    val blockSize = subblockSize * nSubblk
    val superblockSize = blockSize * nBlockPerSuperblock
    val nIntraSlot = superblockSize / subblockSize
    val maxCF = 2
    val nWays = p(NL4CacheWays)
    val nSets = p(NL4CacheCapacity) / blockSize / nWays
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      require(isPow2(nSets))
      require(isPow2(nWays))
      require(isPow2(subblockSize))
      require(isPow2(nSubblk))

      val Y = true.B
      val N = false.B
      val debug = false // TODO: disable debug

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
      val blockIdBits = log2Ceil(nBlockPerSuperblock)
      val intraSlotIdBits = log2Ceil(nIntraSlot)
      val intraSlotLenBits = log2Ceil(maxCF) + 1 // range from 1 to maxCF
      assert(intraSlotIdBits == subblkIdBits + blockIdBits)
      assert(subblkIdBits > 0)
      val superblockTagBits = addrWidth - indexBits - intraSlotIdBits - subblockOffsetBits
      // Address layout
      // | SuperblockTag | Index | SlotId = BlockId + SubblkId | SubblockOffset
      val offsetLSB = 0
      val offsetMSB = subblockOffsetBits - 1
      val subblkIdLSB = offsetMSB + 1
      val subblkIdMSB = subblkIdLSB + subblkIdBits - 1
      val blockIdLSB = subblkIdMSB + 1
      val blockIdMSB = blockIdLSB + blockIdBits - 1
      val slotIdLSB = subblkIdLSB
      val slotIdMSB = blockIdMSB
      val indexLSB = blockIdMSB + 1
      val indexMSB = indexLSB + indexBits - 1
      val superblockTagLSB = indexMSB + 1
      val superblockTagMSB = superblockTagLSB + superblockTagBits - 1
      assert(superblockTagMSB + 1 == addrWidth)

      val rst_cnt = RegInit(0.U(log2Up(2 * nSets + 1).W))
      val rst = (rst_cnt < (2 * nSets).U) && !reset.asBool
      when (rst) { rst_cnt := rst_cnt + 1.U }

      val s_idle :: s_gather_write_data :: s_send_bresp :: s_update_meta :: s_tag_read :: s_merge_put_data :: s_pre_data_read :: s_data_read :: s_data_write :: s_wait_ram_awready :: s_do_ram_write :: s_wait_ram_bresp :: s_wait_ram_arready :: s_do_ram_read :: s_data_resp :: Nil = Enum(15)

      val state = RegInit(s_idle)
      // state transitions for each case
      // read hit: s_idle -> s_tag_read -> s_data_read -> s_data_resp -> s_idle
      // read miss no writeback : s_idle -> s_tag_read -> s_wait_ram_arready -> s_do_ram_read -> s_data_write -> s_update_meta -> s_idle
      // read miss writeback : s_idle -> s_tag_read -> s_pre_data_read -> s_data_read -> s_wait_ram_awready -> s_do_ram_write -> s_wait_ram_bresp
      //                              -> s_wait_ram_arready -> s_do_ram_read -> s_data_write -> s_update_meta -> s_idle
      // write hit: s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_data_read -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      // write miss no writeback : s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_wait_ram_arready -> s_do_ram_read
      //                                  -> s_merge_put_data -> s_data_write -> s_update_meta -> s_idle
      // write miss writeback : s_idle -> s_gather_write_data ->  s_send_bresp -> s_tag_read -> s_pre_data_read -> s_data_read -> s_wait_ram_awready -> s_do_ram_write -> s_wait_ram_bresp
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
        data = Vec(nWays, new L4MetadataEntry(superblockTagBits, nSubblk, intraSlotIdBits, intraSlotLenBits))
      )

      val meta_raddr = Mux(in.ar.fire(), in_ar.addr, addr)
      val meta_array_wen = rst || state === s_update_meta
      val meta_ridx = meta_raddr(indexMSB, indexLSB)

      val meta_rdata = meta_array.read(meta_ridx, !meta_array_wen)

      val subblkId = addr(subblkIdMSB, subblkIdLSB)
      val blockId = addr(blockIdMSB, blockIdLSB)
      val slotId = Cat(blockId, subblkId)
      val slotLen = 1.U(intraSlotLenBits.W) // TODO: always 1-slot size
      val idx = addr(indexMSB, indexLSB)
      val superblockTag = addr(superblockTagMSB, superblockTagLSB)

      def wayMap[T <: Data](f: Int => T) = VecInit((0 until nWays).map(f))
      def subblkMap[T <: Data](f: Int => T) = VecInit((0 until nSubblk).map(f))

      val block_vb_rdata = wayMap((w: Int) => meta_rdata(w).valid).asUInt
      val superblockTag_rdata = wayMap((w: Int) => meta_rdata(w).superblockTag)
      val subblock_vb_rdata = wayMap((w: Int) => meta_rdata(w).subblockValid)
      val subblock_db_rdata = wayMap((w: Int) => meta_rdata(w).subblockDirty)
      val intraSlotId_rdata = wayMap((w: Int) => meta_rdata(w).intraSlotId)
      val intraSlotLen_rdata = wayMap((w: Int) => meta_rdata(w).intraSlotLen)

      val tag_eq_way = wayMap((w: Int) => superblockTag_rdata(w) === superblockTag)
      val tag_match_way = wayMap((w: Int) => tag_eq_way(w) && block_vb_rdata(w)).asUInt
      val hit_way = Wire(Bits())
      hit_way := 0.U
      (0 until nWays).foreach(i => when (tag_match_way(i)) { hit_way := i.U }) // TODO: multiple-hit
      val superblock_hit = tag_match_way.orR // superblock hit/miss
      val superblock_read_miss = !superblock_hit && ren
      val superblock_write_miss = !superblock_hit && wen

      val slot_eq_subblk = subblkMap((w: Int) => (intraSlotId_rdata(hit_way)(w) <= slotId) && (intraSlotId_rdata(hit_way)(w) + intraSlotLen_rdata(hit_way)(w) > slotId))
      val slot_match_subblk = subblkMap((w: Int) => slot_eq_subblk(w) && subblock_vb_rdata(hit_way)(w)).asUInt
      val hit_slot = Wire(UInt(subblkIdBits.W))
      hit_slot := 0.U
      (0 until nSubblk).foreach(i => when (slot_match_subblk(i)) { hit_slot := i.U })
      val subblock_hit = superblock_hit && slot_match_subblk.orR
      val subblock_miss = superblock_hit && !slot_match_subblk.orR

      val repl_slot_lfsr = LFSR(log2Ceil(nSubblk) + 1, state === s_update_meta && !subblock_hit)
      val repl_slot_enable = (state === s_idle && in.ar.fire()) || (state === s_send_bresp && in.b.fire())
      val repl_slot = RegEnable(next = if(nSubblk == 1) 0.U else (repl_slot_lfsr(log2Ceil(nSubblk) - 1, 0) % nSubblk.U), // TODO: remove mod()
        init = 0.U, enable = repl_slot_enable)
      // val repl_slot = 0.U // TODO: always repl slot-0
      val update_slot = Mux(subblock_hit, hit_slot, repl_slot)

      val subblock_read_hit = subblock_hit && ren // situation 1: superblock hit, subblock hit
      val subblock_write_hit = subblock_hit && wen // situation 2
      val subblock_read_miss_no_writeback = subblock_miss && ren && !subblock_db_rdata(hit_way)(repl_slot) // situation 3: superblock hit, but subblk mismatch, no writeback
      val subblock_write_miss_no_writeback = subblock_miss && wen && !subblock_db_rdata(hit_way)(repl_slot) // situation 4
      val subblock_read_miss_writeback = subblock_miss && ren && subblock_vb_rdata(hit_way)(repl_slot) && subblock_db_rdata(hit_way)(repl_slot)
      // situation 9: superblock hit, but subblk mismatch, and need writeback
      val subblock_write_miss_writeback = subblock_miss && wen && subblock_vb_rdata(hit_way)(repl_slot) && subblock_db_rdata(hit_way)(repl_slot)
      // situation 10

      // use random replacement
      // val repl_way_lfsr = LFSR(log2Ceil(nWays), state === s_update_meta && !superblock_hit)
      // val repl_way_enable = (state === s_idle && in.ar.fire()) || (state === s_send_bresp && in.b.fire())
      // val repl_way = RegEnable(next = if(nWays == 1) 0.U else repl_way_lfsr(log2Ceil(nWays) - 1, 0),
        // init = 0.U, enable = repl_way_enable)
      val repl_way = 0.U // TODO: enable 1-way
      val update_way = Mux(superblock_hit, hit_way, repl_way)

      // valid and dirty
      // writeback in the block level
      val superblock_need_writeback = !superblock_hit && subblock_db_rdata(repl_way).orR
      // val superblock_need_writeback = !superblock_hit && true.B // TODO: try always writeback
      val subblock_need_writeback = subblock_read_miss_writeback || subblock_write_miss_writeback

      val wb_superblockTag = Mux(superblock_need_writeback, superblockTag_rdata(repl_way), superblockTag)
      val wb_subblkId = Wire(UInt(subblkIdBits.W))
      val wb_blockId = Wire(UInt(blockIdBits.W))
      val wb_slotId = RegInit(0.U((subblkIdBits).W))
      wb_blockId := Mux(superblock_need_writeback, intraSlotId_rdata(repl_way)(wb_slotId)(subblkIdBits + blockIdBits - 1, subblkIdBits),
                                                   intraSlotId_rdata(hit_way)(wb_slotId)(subblkIdBits + blockIdBits - 1, subblkIdBits))
      wb_subblkId := Mux(superblock_need_writeback, intraSlotId_rdata(repl_way)(wb_slotId)(blockIdBits - 1, 0),
                                                   intraSlotId_rdata(hit_way)(wb_slotId)(blockIdBits - 1, 0))
      val wb_addr = Cat(wb_superblockTag, Cat(idx, Cat(wb_blockId, Cat(wb_subblkId, 0.U(subblockOffsetBits.W)))))

      val block_read_miss_writeback = superblock_read_miss && superblock_need_writeback // situation 5
      val block_read_miss_no_writeback = superblock_read_miss && !superblock_need_writeback // situation 6
      val block_write_miss_writeback = superblock_write_miss && superblock_need_writeback // situation 7
      val block_write_miss_no_writeback = superblock_write_miss && !superblock_need_writeback // situation 8

      val need_data_read = subblock_read_hit || subblock_write_hit || // subblk hit
                           block_read_miss_writeback || block_write_miss_writeback || // block miss, need writeback
                           subblock_read_miss_writeback || subblock_write_miss_writeback // subblock miss, need writeback
                            // situation 1, 2, 5, 7, 9, 10
      val no_need_data_read = subblock_read_miss_no_writeback || subblock_write_miss_no_writeback ||
                              block_read_miss_no_writeback || block_write_miss_no_writeback
                            // situation: 3, 4, 6, 8

      val data_buf = Reg(Vec(outerSubblkBeats, UInt(outerBeatSize.W)))

      val comp_en = block_read_miss_no_writeback // TODO: only support situation 6
      val refill_done_decl = Wire(Bool()) // forward declare refill_done
      val (prefetch_cnt, prefetch_done) = Counter(refill_done_decl, maxCF)
      val rf_slotId = slotId + prefetch_cnt
      val rf_subblkId = rf_slotId(subblkIdBits - 1, 0)
      val rf_blockId = rf_slotId(subblkIdBits + blockIdBits - 1, subblkIdBits)
      val rf_addr = Cat(superblockTag, Cat(idx, Cat(rf_blockId, Cat(rf_subblkId, 0.U(subblockOffsetBits.W)))))
      val compressor = Module(new ZeroLeadingCompressor(outerSubblkBeats, outerBeatSize, intraSlotLenBits))
      compressor.io.data := data_buf
      compressor.io.data_lastbeat := out_r.data
      compressor.io.first_line := prefetch_cnt === 0.U
      compressor.io.last_line  := prefetch_done
      compressor.io.line_ready := refill_done_decl
      val comp_data_buf = compressor.io.comp_data
      val comp_slotLen = compressor.io.comp_len // TODO: if slotId == 0x11, slotLen <= 1

      when (state === s_tag_read) {
        log("req: isread %x iswrite %x addr %x idx %d superblockTag %x blockId %x subblkId %x superblockHit %d subblkHit %d hit_way %x",
            ren, wen,
            addr,
            idx, superblockTag, blockId, subblkId,
            superblock_hit, subblock_hit, hit_way)
        when (!superblock_hit) {
          log("\tSB miss repl_way %x repl_valid %x repl_slotValid %x repl_slotDirty %x repl_slotId %x needwb %x wbaddr %x",
              repl_way, block_vb_rdata(repl_way), subblock_vb_rdata(repl_way), subblock_db_rdata(repl_way), intraSlotId_rdata(repl_way).asUInt, superblock_need_writeback, wb_addr)
        } .otherwise {
          log("\tSB hit hit_way %x hit_slotValid %x hit_slotDirty %x hit_slotId %x needwb %x wbaddr %x",
              hit_way, subblock_vb_rdata(hit_way), subblock_db_rdata(hit_way), intraSlotId_rdata(hit_way).asUInt, subblock_need_writeback, wb_addr)
        }
        log("\tSUB hit %x hit_slot %x repl_slot %x",
            subblock_hit, hit_slot, repl_slot)

        // check for cross cache line bursts
        assert(inner_end_beat < innerDataBeats.U, "cross cache line bursts detected")

        when (need_data_read) {
          // insert a cycle for wb_slotId to ready
          when (superblock_need_writeback) {
            wb_slotId := 0.U // scan from subblockId 0
            state := s_pre_data_read
          } .elsewhen (subblock_need_writeback) {
            wb_slotId := repl_slot
            state := s_pre_data_read
          } .otherwise {
            state := s_data_read
          }
        } .elsewhen (no_need_data_read) {
          // no need to write back, directly refill data
          state := s_wait_ram_arready
        } .otherwise {
          assert(N, "Unexpected condition in s_tag_read")
        }
      }

      when (state === s_pre_data_read) {
        state := s_data_read
        when (superblock_need_writeback) {
          log("super writeback start: idx %d way %x addr %x slotId %x wb_blockId %x wb_subblkId %x ", idx, repl_way, wb_addr, wb_slotId, wb_blockId, wb_subblkId)
        } .elsewhen (subblock_need_writeback) {
          log("sub writeback start: idx %d way %x addr %x slotId %x wb_blockId %x wb_subblkId %x ", idx, hit_way, wb_addr, wb_slotId, wb_blockId, wb_subblkId)
        }
      }

      // ###############################################################
      // #                  data array read/write                      #
      // ###############################################################
      // data array read idx and beat number are different for situation (5, 7) and (1, 2)
      val data_read_way = Mux(superblock_hit, hit_way, repl_way)
      // incase it overflows
      val data_read_cnt = RegInit(0.U((innerBeatIndexBits + 1).W))
      val data_read_is_last_beat = data_read_cnt === innerDataBeats.U
      val data_read_valid = (state === s_pre_data_read && need_data_read) || (state === s_data_read && !data_read_is_last_beat)
      val data_read_idx = Mux(superblock_need_writeback || subblock_need_writeback, (idx << (innerBeatIndexBits + subblkIdBits)) | (wb_slotId << innerBeatIndexBits) | data_read_cnt,
                              (idx << (innerBeatIndexBits + subblkIdBits)) | (hit_slot << innerBeatIndexBits) | data_read_cnt)
      val dout = Wire(Vec(split, UInt(outerBeatSize.W)))
      val decompressor = Module(new ZeroLeadingDecompressor(outerSubblkBeats, outerBeatSize, intraSlotLenBits))
      decompressor.io.comp_data := data_buf
      decompressor.io.comp_slotLen := slotLen
      decompressor.io.offset := slotId - intraSlotId_rdata(hit_way)(hit_slot)
      decompressor.io.ready := data_read_is_last_beat
      val decomp_data_buf = decompressor.io.decomp_data

      val data_write_way = Mux(superblock_hit, hit_way, repl_way)
      val data_write_cnt = Wire(UInt())
      val data_write_valid = state === s_data_write
      val data_write_idx = Mux(subblock_hit, (idx << (innerBeatIndexBits + subblkIdBits)) | (hit_slot << innerBeatIndexBits) | data_write_cnt,
                                             (idx << (innerBeatIndexBits + subblkIdBits)) | (repl_slot << innerBeatIndexBits) | data_write_cnt)
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
          val sel_slotId = Mux(subblock_hit, hit_slot, repl_slot)
          log("write data array: %d idx %d slotId %d way %d cnt %d data %x\n",
            i.U, idx, sel_slotId, data_write_way, data_write_cnt, din(i))
          data_array.write(data_write_idx, VecInit(Seq.fill(nWays) { din(i) }), (0 until nWays).map(data_write_way === _.asUInt))
        }
        dout(i) := data_array.read(data_read_idx, data_read_valid && !data_write_valid)(data_read_way)
        when (RegNext(data_read_valid, N)) {
          val sel_slotId = Mux(superblock_need_writeback || subblock_need_writeback, wb_slotId, hit_slot)
          log("read data array: %d idx %d slotId %d way %d super_wb %x sub_wb %x cnt %d data %x\n",
            i.U, RegNext(idx), RegNext(sel_slotId), RegNext(data_read_way), RegNext(superblock_need_writeback), RegNext(subblock_need_writeback), RegNext(data_read_cnt), dout(i))
        }
      }

      // s_data_read
      // read miss or need write back
      when (data_read_valid) {
        data_read_cnt := data_read_cnt + 1.U
      }
      when (state === s_data_read) {
        for (i <- 0 until split) {
          data_buf(((data_read_cnt - 1.U) << splitBits) + i.U) := dout(i)
        }
        when (data_read_is_last_beat) {
          when (slotLen > 1.U) {
            // data_buf := decomp_data_buf
            (0 until outerSubblkBeats).map((w: Int) => data_buf(w) := decomp_data_buf(w))
          }
          data_read_cnt := 0.U
          when (subblock_read_hit) {
            state := s_data_resp
          } .elsewhen (block_read_miss_writeback || block_write_miss_writeback || subblock_read_miss_writeback || subblock_write_miss_writeback) {
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
      val update_metadata = Wire(Vec(nWays, new L4MetadataEntry(superblockTagBits, nSubblk, intraSlotIdBits, intraSlotLenBits)))
      val rst_metadata = Wire(Vec(nWays, new L4MetadataEntry(superblockTagBits, nSubblk, intraSlotIdBits, intraSlotLenBits)))

      for (i <- 0 until nWays) {
        val metadata = rst_metadata(i)
        metadata.valid := false.B
        metadata.superblockTag := 0.U
        metadata.subblockValid := 0.U(nSubblk.W)
        metadata.subblockDirty := 0.U(nSubblk.W)
        metadata.intraSlotId := VecInit(Seq.fill(nSubblk) { 0.U(intraSlotIdBits.W) })
        metadata.intraSlotLen := VecInit(Seq.fill(nSubblk) { 0.U(intraSlotLenBits.W) })
      }

      for (i <- 0 until nWays) {
        val metadata = update_metadata(i)
        val is_update_way = update_way === i.U
        when (is_update_way) {
          metadata.valid := true.B
          metadata.superblockTag := superblockTag
          metadata.subblockValid := Mux(superblock_hit, subblock_vb_rdata(i) | (1.U << update_slot),
                                    1.U << update_slot)
          metadata.subblockDirty := Mux(superblock_hit, subblock_db_rdata(i) | (wen << update_slot),
                                    (wen << update_slot)) // TODO: more saving dirty policy
          val reset_intraSlotId  = VecInit(Seq.fill(nSubblk) { 0.U(intraSlotIdBits.W) })
          metadata.intraSlotId   := Mux(superblock_hit, intraSlotId_rdata(i), reset_intraSlotId)
          metadata.intraSlotId(update_slot) := slotId
          val reset_intraSlotLen = VecInit(Seq.fill(nSubblk) { 0.U(intraSlotLenBits.W) })
          metadata.intraSlotLen  := Mux(superblock_hit, intraSlotLen_rdata(i), reset_intraSlotLen)
          when (!subblock_hit) { // TODO: why this condition? I forget
            metadata.intraSlotLen(update_slot) := Mux(comp_en, comp_slotLen, 1.U)
          }
        } .otherwise {
          metadata.valid := block_vb_rdata(i)
          metadata.superblockTag := superblockTag_rdata(i)
          metadata.subblockValid := subblock_vb_rdata(i)
          metadata.subblockDirty := subblock_db_rdata(i)
          metadata.intraSlotId   := intraSlotId_rdata(i)
          metadata.intraSlotLen  := intraSlotLen_rdata(i)
        }
      }

      val meta_array_widx = Mux(rst, rst_cnt, idx)
      val meta_array_wdata = Mux(rst, rst_metadata, update_metadata)

      when (meta_array_wen) {
        meta_array.write(meta_array_widx, meta_array_wdata)
        when (!rst) {
          log("update_meta: idx %d superblockTag %x valid %x subValid %x subDirty %x intraSlotId %x intraSlotLen %x update_way %d update_slot %d\n",
              idx, update_metadata(update_way).superblockTag, update_metadata(update_way).valid,
              update_metadata(update_way).subblockValid, update_metadata(update_way).subblockDirty,
              update_metadata(update_way).intraSlotId.asUInt, update_metadata(update_way).intraSlotLen.asUInt,
              update_way, update_slot)
          log("update_meta1: comp_en %x comp_slotLen %x", comp_en, comp_slotLen)
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
        log("data_writeback: idx %d superblockTag %x wb_addr %x cnt %x wb_done %x slotId %x wbBlkId %x wbSubId %x subValid %x subDirty %x wb_strb %x wb_data %x\n", idx, wb_superblockTag, wb_addr, wb_cnt, wb_done, wb_slotId, wb_blockId, wb_subblkId, subblock_vb_rdata(update_way), subblock_db_rdata(update_way), out_w.strb, out_w.data)
      }
      when (!reset.asBool && out.w.fire()) {
        bothHotOrNoneHot(wb_done, out_w.last, "L4 write back error")
      }

      // write address channel signals
      out_aw.id := 0.U(outerIdWidth.W)
      out_aw.addr := wb_addr
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
      val out_w_strb = Mux(subblock_db_rdata(update_way)(wb_slotId), Fill(outerBeatBytes, 1.U(1.W)),
                            Fill(outerBeatBytes, 0.U(1.W))) // no-op for invalid lines
      out_w.strb := out_w_strb
      out_w.last := wb_cnt === (outerSubblkBeats - 1).U
      //out_w.user := 0.U(5.W)
      out.w.valid := state === s_do_ram_write

      when (state === s_wait_ram_bresp && out.b.fire()) {
        when (block_read_miss_writeback || block_write_miss_writeback) {
          when (wb_slotId === (nSubblk - 1).U) {
            // do refill
            state := s_wait_ram_arready
            wb_slotId := 0.U
            log("super writeback all complete: idx %d way %x addr %x slotId %d wb_blockId %x wb_subblkId %x", idx, repl_way, wb_addr, wb_slotId, wb_blockId, wb_subblkId)
          } .otherwise {
            log("super writeback done one for idx %d way %x addr %x slotId %d wb_blockId %x wb_subblkId %x", idx, repl_way, wb_addr, wb_slotId, wb_blockId, wb_subblkId)
            state := s_data_read
            wb_slotId := wb_slotId + 1.U
            assert(wb_slotId <= nSubblk.U)
          }
        } .elsewhen (subblock_need_writeback) {
          // do refill
          state := s_wait_ram_arready
          log("sub writeback complete: idx %d way %x addr %x slotId %d wb_blockId %x wb_subblkId %x", idx, hit_way, wb_addr, wb_slotId, wb_blockId, wb_subblkId)
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
      refill_done_decl := refill_done
      when (state === s_do_ram_read && out.r.fire()) {
        data_buf(refill_cnt) := out_r.data
        log("data_refill: idx %d superblockTag %x blockId %x subblkId %x rf_addr %x rf_cnt %x prefetch_cnt %x prefetch_done %x refill_data %x\n", idx, superblockTag, blockId, subblkId, rf_addr, refill_cnt, prefetch_cnt, prefetch_done, out_r.data)
        when (refill_done) {
          bothHotOrNoneHot(refill_done, out_r.last, "L4 refill error")
          when (ren) {
            when (comp_en && !prefetch_done) {
              state := s_wait_ram_arready
            } .elsewhen (comp_en && prefetch_done) {
              data_buf := comp_data_buf
              state := s_data_write
              prefetch_cnt := 0.U
            } .otherwise {
              state := s_data_write
              prefetch_cnt := 0.U
            }
          } .otherwise {
            prefetch_cnt := 0.U // restore prefetch states
            state := s_merge_put_data
          }
        }
      }

      // read address channel signals
      out_ar.id := 0.U(outerIdWidth.W)
      out_ar.addr := rf_addr
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
        log("data_resp: idx %d superblockTag %x blockId %x subblkId %x req_addr %x cnt %x resp_data %x\n", idx, superblockTag, blockId, subblkId, addr, resp_curr_beat, resp_data.asUInt)
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
      L4PerfAccumulator("l4_req_superblock_hit", state === s_tag_read && superblock_hit)
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
