package utils

import chisel3._
import chisel3.util._

class DataModuleTemplate[T <: Data](gen: T, numEntries: Int, numRead: Int, numWrite: Int) extends Module {
  val io = IO(new Bundle {
    val raddr = Vec(numRead,  Input(UInt(log2Up(numEntries).W)))
    val rdata = Vec(numRead,  Output(gen))
    val wen   = Vec(numWrite, Input(Bool()))
    val waddr = Vec(numWrite, Input(UInt(log2Up(numEntries).W)))
    val wdata = Vec(numWrite, Input(gen))
  })

  val data = Mem(numEntries, gen)

  // read ports
  for (i <- 0 until numRead) {
    io.rdata(i) := data(io.raddr(i))
  }

  // write ports
  // val waddr_dec = VecInit(io.waddr.map(addr => UIntToOH(addr)(numEntries - 1, 0)))
  // val waddr_dec_with_en = VecInit(io.wen.zip(waddr_dec).map{case (en, addr) => Fill(numEntries, en) & addr})

  // val wen_dec = VecInit((0 until numEntries).map(i => Cat(waddr_dec_with_en.map(en => en(i))).orR))
  // val wdata_dec = VecInit((0 until numEntries).map(i => waddr_dec_with_en.zip(io.wdata).map{ case (en, data) =>
  //   Fill(gen.getWidth, en) & data.asUInt
  // }.reduce(_ | _).asTypeOf(gen)))

  // waddr_dec.suggestName("waddr_dec")
  // waddr_dec_with_en.suggestName("waddr_dec_with_en")
  // wen_dec.suggestName("wen_dec")
  // wdata_dec.suggestName("wdata_dec")

  // for (i <- 0 until numEntries) {
  //   when (wen_dec(i)) {
  //     data(i) := wdata_dec(i)
  //   }
  // }

  // below is the write ports with priorities
  for (i <- 0 until numWrite) {
    when (io.wen(i)) {
      data(io.waddr(i)) := io.wdata(i)
    }
  }

  // DataModuleTemplate should not be used when there're any write conflicts
  for (i <- 0 until numWrite) {
    for (j <- i+1 until numWrite) {
      assert(!(io.wen(i) && io.wen(j) && io.waddr(i) === io.waddr(j)))
    }
  }
}
