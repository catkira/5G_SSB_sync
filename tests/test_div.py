import os
import logging
import os
import pytest

import numpy as np
import cocotb
import cocotb_test.simulator
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


CLK_PERIOD_NS = 8
CLK_PERIOD_S = CLK_PERIOD_NS * 0.000000001
tests_dir = os.path.abspath(os.path.dirname(__file__))
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', 'hdl'))

def _twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return int(val)

class TB(object):
    def __init__(self, dut):
        self.dut = dut
        self.INPUT_WIDTH = int(dut.INPUT_WIDTH.value)
        self.RESULT_WIDTH = int(dut.RESULT_WIDTH.value)
        self.PIPELINED = int(dut.PIPELINED.value)

        self.log = logging.getLogger('cocotb.tb')
        self.log.setLevel(logging.DEBUG)

        cocotb.start_soon(Clock(self.dut.clk_i, CLK_PERIOD_NS, units='ns').start())

    async def cycle_reset(self):
        self.dut.reset_ni.setimmediatevalue(1)
        await RisingEdge(self.dut.clk_i)
        self.dut.reset_ni.value = 0
        await RisingEdge(self.dut.clk_i)
        self.dut.reset_ni.value = 1
        await RisingEdge(self.dut.clk_i)

@cocotb.test()
async def simple_test(dut):
    tb = TB(dut)

    dut.valid_i.value = 0
    await tb.cycle_reset()

    numerator = np.random.randint(0, 2**(tb.INPUT_WIDTH) - 1)
    denominator = np.random.randint(1, 2**(tb.INPUT_WIDTH) - 1)

    numerator = 3698210746
    denominator = 9845
    # numerator = 2775
    # denominator = 48908
    dut.numerator_i.value = numerator
    dut.denominator_i.value = denominator
    dut.valid_i.value = 1

    await RisingEdge(dut.clk_i)
    dut.valid_i.value = 0

    clk_cnt = 0
    max_clk_cnt = 1000
    rx_cnt = 0
    max_rx_cnt = 500
    while (clk_cnt < max_clk_cnt) and (rx_cnt < max_rx_cnt):
        await RisingEdge(dut.clk_i)
        clk_cnt += 1

        if (dut.valid_o.value == 1):
            result = dut.result_o.value.integer
            # print(f'{numerator} / {denominator} = {result}')
            assert np.floor(numerator / denominator) == result
            rx_cnt += 1

            numerator = np.random.randint(0, 2**(tb.INPUT_WIDTH) - 1)
            denominator = np.random.randint(1, 2**(tb.INPUT_WIDTH) - 1)
            dut.numerator_i.value = numerator
            dut.denominator_i.value = denominator
            dut.valid_i.value = 1
        else:
            dut.valid_i.value = 0

    if clk_cnt == max_clk_cnt:
        print("no result received!")
    

@pytest.mark.parametrize("INPUT_WIDTH", [16, 32])
@pytest.mark.parametrize("RESULT_WIDTH", [16, 32])
@pytest.mark.parametrize("PIPELINED", [0, 1])
def test(INPUT_WIDTH, RESULT_WIDTH, PIPELINED):
    dut = 'div'
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(rtl_dir, f'{dut}.sv')
    ]
    includes = []

    parameters = {}
    parameters['INPUT_WIDTH'] = INPUT_WIDTH
    parameters['RESULT_WIDTH'] = RESULT_WIDTH
    parameters['PIPELINED'] = PIPELINED

    parameters_dir = parameters.copy()
    sim_build='sim_build/' + '_'.join(('{}={}'.format(*i) for i in parameters_dir.items()))
    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        includes=includes,
        toplevel=toplevel,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        testcase='simple_test',
        force_compile=True,
        waves=True
    )

if __name__ == '__main__':
    test(INPUT_WIDTH = 32, RESULT_WIDTH = 16, PIPELINED = 1)
