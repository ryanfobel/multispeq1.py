from nose.tools import with_setup
import numpy as np
import multispeq1 as tr


def setup_func():
    global proxy
    proxy = tr.SerialProxy()


def teardown_func():
    global proxy
    del proxy


@with_setup(setup_func, teardown_func)
def test_float_performance():
    '''
    Compare float performance against empirical results.
    '''
    emperical_flop_us = 0.041857543945312499  # Empirical mean of 1000 tests

    N = 8 << 10
    flop_us = proxy.benchmark_flops_us(N) / float(N)
    np.testing.assert_allclose(emperical_flop_us, flop_us, rtol=1e-2)


@with_setup(setup_func, teardown_func)
def test_int_performance():
    '''
    Compare integer performance against empirical results.
    '''
    emperical_iop_us = 0.041865478515624999 # Empirical mean of 1000 tests

    N = 8 << 10
    iop_us = proxy.benchmark_iops_us(N) / float(N)
    np.testing.assert_allclose(emperical_iop_us, iop_us, rtol=1e-2)
