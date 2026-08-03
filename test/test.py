import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_wms_basic(dut):
    dut._log.info("Starting WMS debounced basic test")

    # Start 50MHz clock (20ns period)
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())

    # Initialize inputs and pulse active-low reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # -------------------------------------------------------------------------
    # Test 1: Set Lower Tank Sensor HIGH (LT_SIGNAL = ui_in[6])
    # -------------------------------------------------------------------------
    dut.ui_in.value = 0b01000000 # LT_SIGNAL = 1, UP_10 = 0
    
    # NOTE: If SIMULATION macro isn't passed to iverilog, debouncer needs cycles.
    # We wait 20 cycles to ensure stability across simulation stages.
    await ClockCycles(dut.clk, 25)

    dut._log.info(f"uo_out state: {dut.uo_out.value}")
    assert dut.uo_out[0].value == 1, "Motor failed to start when lower tank had water and upper tank was empty"

    # -------------------------------------------------------------------------
    # Test 2: Upper Tank reaches 100% (UP_100 = ui_in[5])
    # -------------------------------------------------------------------------
    dut.ui_in.value = 0b01100000 # LT_SIGNAL = 1, UP_100 = 1
    await ClockCycles(dut.clk, 25)

    assert dut.uo_out[0].value == 0, "Motor failed to stop when upper tank reached 100%"

    dut._log.info("WMS basic test passed successfully!")
