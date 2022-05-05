/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package top

import chipsalliance.rocketchip.config
import chisel3._
import device._
import difftest._
import freechips.rocketchip.amba.axi4.{AXI4EdgeParameters, AXI4Fragmenter, AXI4IdIndexer, AXI4MasterNode, AXI4ToTL, AXI4UserYanker, AXI4Xbar}
import freechips.rocketchip.devices.tilelink.{DevNullParams, TLError}
import freechips.rocketchip.diplomacy.{AddressSet, InModuleBody, LazyModule, LazyModuleImp}
import freechips.rocketchip.tilelink.{TLFIFOFixer, TLToAXI4, TLWidthWidget, TLXbar}
import system.SoCParamsKey

class SimMMIO(edge: AXI4EdgeParameters)(implicit p: config.Parameters) extends LazyModule {

  val node = AXI4MasterNode(List(edge.master))

  val flash = LazyModule(new AXI4Flash(Seq(AddressSet(0x1ffff80000L, 0x3ffff))))
  val uart = LazyModule(new AXI4UART(Seq(AddressSet(0x1f10050000L, 0xf))))
  val vga = LazyModule(new AXI4VGA(
    sim = false,
    fbAddress = Seq(AddressSet(0x1f50000000L, 0x3fffffL)),
    ctrlAddress = Seq(AddressSet(0x1f40001000L, 0x7L))
  ))
  val sd = LazyModule(new AXI4DummySD(Seq(AddressSet(0x1f40002000L, 0xfff))))
  val intrGen = LazyModule(new AXI4IntrGenerator(Seq(AddressSet(0x1f10060000L, 0x0000ffffL))))

  val axiBus = AXI4Xbar()
  val paddrBits = p(SoCParamsKey).PAddrBits
  val paddrRange = AddressSet(0x00000000L, (1L << paddrBits) - 1)
  val peripheralRange = AddressSet(0x1f00000000L, 0xffffffffL)
  val errorDev = LazyModule(new TLError(
    params = DevNullParams(
      address = paddrRange.subtract(peripheralRange),
      maxAtomic = 8,
      maxTransfer = 128
    ),
    beatBytes = 8
  ))

  uart.node := axiBus
  vga.node :*= axiBus
  flash.node := axiBus
  sd.node := axiBus
  intrGen.node := axiBus

  val tlBus = TLXbar()
  tlBus :=
    TLFIFOFixer() :=
    TLWidthWidget(32) :=
    AXI4ToTL() :=
    AXI4UserYanker(Some(1)) :=
    AXI4IdIndexer(5) :=
    node
  errorDev.node := tlBus
  axiBus := TLToAXI4() := tlBus

  val io_axi4 = InModuleBody {
    node.makeIOs()
  }

  lazy val module = new LazyModuleImp(this){
    val io = IO(new Bundle() {
      val uart = new UARTIO
      val interrupt = new IntrGenIO
    })
    io.uart <> uart.module.io.extra.get
    io.interrupt <> intrGen.module.io.extra.get
  }

}
