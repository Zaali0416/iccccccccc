import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

@cocotb.test()
async def test_wms_basic(dut):
    dut._log.info("Starting WMS basic test")

    # Start 50MHz clock (20ns period)
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())

    # Initialize inputs
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    # Reset for 10 clock cycles
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Test 1: Lower tank has water (ui_in[6]=1), Upper tank 0% (ui_in[0]=0)
    # Expected: Motor should turn ON (uo_out[0] = 1)
    dut.ui_in.value = 0b01000000 # LT_SIGNAL = 1
    await ClockCycles(dut.clk, 5)

    dut._log.info(f"uo_out state: {dut.uo_out.value}")
    assert dut.uo_out[0].value == 1, "Motor failed to start when lower tank had water and upper tank was empty"

    # Test 2: Upper tank full (ui_in[5]=1)
    # Expected: Motor should turn OFF (uo_out[0] = 0)
    dut.ui_in.value = 0b01100000 # LT_SIGNAL = 1, UP_100 = 1
    await ClockCycles(dut.clk, 5)

    assert dut.uo_out[0].value == 0, "Motor failed to stop when upper tank reached 100%"

    dut._log.info("WMS basic test passed successfully!")
