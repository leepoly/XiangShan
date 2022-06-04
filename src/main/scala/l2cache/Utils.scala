package l4cache

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.{Field, Parameters}
import utils.GTimer

object CheckOneHot {
  def apply(in: Seq[Bool]): Unit = {
    val value = VecInit(in).asUInt
    val length = in.length
    def bitMap[T <: Data](f: Int => T) = VecInit((0 until length).map(f))
    val bitOneHot = bitMap((w: Int) => value === (1 << w).asUInt(length.W)).asUInt
    val oneHot = bitOneHot.orR
    val noneHot = value === 0.U
    assert(oneHot || noneHot)
  }
}

// a and b must be both hot or none hot
object bothHotOrNoneHot {
  def apply(a: Bool, b: Bool, str: String): Unit = {
    val cond = (a === b)
      assert(cond, str)
  }
}

object L4PerfAccumulator {
  def apply(perfName: String, perfCnt: UInt)(implicit p: Parameters) = {
      val counter = RegInit(0.U(64.W))
      val next_counter = counter + perfCnt
      counter := next_counter
      when (GTimer() % 1000000.U === 0.U) {
        printf(p"[l4counter] $perfName, $next_counter\n")
      }
  }
}
