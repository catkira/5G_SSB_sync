import numpy as np
import scipy
import os
import pytest
import logging
import importlib
import matplotlib.pyplot as plt
import os
import importlib.util

import cocotb
import cocotb_test.simulator
from cocotb.clock import Clock
from cocotb.triggers import Timer
from cocotb.triggers import RisingEdge

import py3gpp
import sigmf

CLK_PERIOD_NS = 8
CLK_PERIOD_S = CLK_PERIOD_NS * 0.000000001
tests_dir = os.path.abspath(os.path.dirname(__file__))
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', 'hdl'))

class TB(object):
    def __init__(self, dut):
        self.dut = dut
        self.IN_DW = int(dut.IN_DW.value)
        self.OUT_DW = int(dut.OUT_DW.value)
        self.TAP_DW = int(dut.TAP_DW.value)
        self.PSS_LEN = int(dut.PSS_LEN.value)
        self.ALGO = int(dut.ALGO.value)
        self.USE_TAP_FILE = int(dut.USE_TAP_FILE.value)
        self.TAP_FILE = dut.TAP_FILE.value

        self.log = logging.getLogger('cocotb.tb')
        self.log.setLevel(logging.DEBUG)

        tests_dir = os.path.abspath(os.path.dirname(__file__))
        model_dir = os.path.abspath(os.path.join(tests_dir, '../model/PSS_correlator.py'))
        spec = importlib.util.spec_from_file_location('PSS_correlator', model_dir)
        foo = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(foo)

        if self.USE_TAP_FILE:
            self.TAP_FILE = os.environ['TAP_FILE']
            self.PSS_LOCAL = 0
        else:
            self.TAP_FILE = ""
            self.PSS_LOCAL =  int(dut.PSS_LOCAL.value)        
        self.model = foo.Model(self.IN_DW, self.OUT_DW, self.TAP_DW, self.PSS_LEN, self.PSS_LOCAL, self.ALGO, self.USE_TAP_FILE, self.TAP_FILE)

        cocotb.start_soon(Clock(self.dut.clk_i, CLK_PERIOD_NS, units='ns').start())
        cocotb.start_soon(self.model_clk(CLK_PERIOD_NS, 'ns'))

    async def model_clk(self, period, period_units):
        timer = Timer(period, period_units)
        while True:
            self.model.tick()
            await timer

    async def cycle_reset(self):
        self.dut.s_axis_in_tvalid.value = 0
        self.dut.reset_ni.setimmediatevalue(1)
        await RisingEdge(self.dut.clk_i)
        self.dut.reset_ni.value = 0
        await RisingEdge(self.dut.clk_i)
        self.dut.reset_ni.value = 1
        await RisingEdge(self.dut.clk_i)
        self.model.reset()

@cocotb.test()
async def simple_test(dut):
    tb = TB(dut)
    handle = sigmf.sigmffile.fromfile('../../tests/30720KSPS_dl_signal.sigmf-data')
    waveform = handle.read_samples()
    fs = 30720000
    CFO = int(os.getenv('CFO'))
    print(f'CFO = {CFO} Hz')
    waveform *= np.exp(np.arange(len(waveform))*1j*2*np.pi*CFO/fs)
    waveform /= max(waveform.real.max(), waveform.imag.max())
    waveform = scipy.signal.decimate(waveform, 16, ftype='fir')
    waveform /= max(waveform.real.max(), waveform.imag.max())
    waveform *= 2 ** (tb.IN_DW // 2 - 1) - 1
    waveform = waveform.real.astype(int) + 1j*waveform.imag.astype(int)
    await tb.cycle_reset()

    num_items = 500
    rx_counter = 0
    rx_counter_model = 0
    in_counter = 0
    received = np.empty(num_items, int)
    received_model = np.empty(num_items, int)
    dut.enable_i.value = 1
    while rx_counter < num_items:
        await RisingEdge(dut.clk_i)
        data = (((int(waveform[in_counter].imag)  & (2 ** (tb.IN_DW // 2) - 1)) << (tb.IN_DW // 2)) \
              + ((int(waveform[in_counter].real)) & (2 ** (tb.IN_DW // 2) - 1))) & (2 ** tb.IN_DW - 1)
        dut.s_axis_in_tdata.value = data
        dut.s_axis_in_tvalid.value = 1
        tb.model.set_data(data)
        in_counter += 1

        if dut.m_axis_out_tvalid == 1:
            received[rx_counter] = dut.m_axis_out_tdata.value.integer
            # print(f'{rx_counter}: rx hdl {received[rx_counter]}')
            rx_counter  += 1

        if tb.model.data_valid() and rx_counter_model < num_items:
            received_model[rx_counter_model] = tb.model.get_data()
            # print(f'{rx_counter_model}: rx mod {received_model[rx_counter_model]}')
            rx_counter_model += 1

    ssb_start = np.argmax(received)
    print(f'max model {max(received_model)} max hdl {max(received)}')
    if 'PLOTS' in os.environ and os.environ['PLOTS'] == '1':
        _, (ax, ax2) = plt.subplots(2, 1)
        print(f'{type(received.dtype)} {type(received_model.dtype)}')
        ax.plot(np.sqrt(received))
        ax.set_title('hdl')
        ax2.plot(np.sqrt(received_model), 'r-')
        ax.set_title('model')
        ax.axvline(x = ssb_start, color = 'y', linestyle = '--', label = 'axvline - full height')
        plt.show()
    print(f'max correlation is {received[ssb_start]} at {ssb_start}')

    print(f'max model-hdl difference is {max(np.abs(received - received_model))}')
    if tb.ALGO == 0:
        #ok_limit = 0.0001
        #for i in range(len(received)):
        #    assert np.abs((received[i] - received_model[i]) / received[i]) < ok_limit
        for i in range(len(received)):
            assert received[i] == received_model[i]
    else:
        # there is not yet a model for ALGO=1
        pass

    assert ssb_start == 412
    assert len(received) == num_items

# bit growth inside PSS_correlator is a lot, be careful to not make OUT_DW too small !
@pytest.mark.parametrize("ALGO", [0, 1])
@pytest.mark.parametrize("IN_DW", [14, 32])
@pytest.mark.parametrize("OUT_DW", [24, 48])
@pytest.mark.parametrize("TAP_DW", [18, 32])
@pytest.mark.parametrize("CFO", [0, 7500])
@pytest.mark.parametrize("USE_TAP_FILE", [0, 1])
def test(IN_DW, OUT_DW, TAP_DW, ALGO, CFO, USE_TAP_FILE):
    dut = 'PSS_correlator'
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(rtl_dir, f'{dut}.sv')
    ]
    includes = []

    PSS_LEN = 128
    parameters = {}
    parameters['IN_DW'] = IN_DW
    parameters['OUT_DW'] = OUT_DW
    parameters['TAP_DW'] = TAP_DW
    parameters['PSS_LEN'] = PSS_LEN
    parameters['ALGO'] = ALGO
    parameters['USE_TAP_FILE'] = USE_TAP_FILE

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}
    os.environ['CFO'] = str(CFO)
    parameters_no_taps = parameters.copy()
    folder = '_'.join(('{}={}'.format(*i) for i in parameters_no_taps.items()))
    sim_build='sim_build/' + folder
    N_id_2 = 2

    if not USE_TAP_FILE:
        PSS = np.zeros(PSS_LEN, 'complex')
        PSS[0:-1] = py3gpp.nrPSS(N_id_2)
        taps = np.fft.ifft(np.fft.fftshift(PSS))
        taps /= max(taps.real.max(), taps.imag.max())
        taps *= 2 ** (TAP_DW // 2 - 1) - 1
        parameters['PSS_LOCAL'] = 0
        for i in range(len(taps)):
            parameters['PSS_LOCAL'] += ((int(np.imag(taps[i])) & (2 ** (TAP_DW // 2) - 1)) << (TAP_DW * i + TAP_DW // 2)) \
                                    +  ((int(np.real(taps[i])) & (2 ** (TAP_DW // 2) - 1)) << (TAP_DW * i))
    else:
        # every parameter combination needs to have its own TAP_FILE to allow parallel tests!
        parameters['TAP_FILE'] = f'\"../{folder}/PSS_taps_{N_id_2}.hex\"'
        os.environ['TAP_FILE'] = f'{rtl_dir}/../{sim_build}/PSS_taps_{N_id_2}.hex'

        os.makedirs(sim_build, exist_ok=True)
        file_path = os.path.abspath(os.path.join(tests_dir, '../tools/generate_PSS_tap_file.py'))
        spec = importlib.util.spec_from_file_location("generate_PSS_tap_file", file_path)
        generate_PSS_tap_file = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generate_PSS_tap_file)
        generate_PSS_tap_file.main(['--PSS_LEN', str(PSS_LEN),'--TAP_DW', str(TAP_DW), '--N_id_2', str(N_id_2), '--path', sim_build])

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        includes=includes,
        toplevel=toplevel,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
        testcase='simple_test',
        force_compile=True
    )

if __name__ == '__main__':
    os.environ['PLOTS'] = "1"
    test(IN_DW = 32, OUT_DW = 24, TAP_DW = 18, ALGO = 0, CFO = 10000, USE_TAP_FILE = 0)
