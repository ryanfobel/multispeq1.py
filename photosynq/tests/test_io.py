from nose.tools import with_setup, eq_
import photosynq as tr


def setup_func():
    global proxy
    proxy = tr.SerialProxy()


def teardown_func():
    global proxy
    del proxy


@with_setup(setup_func, teardown_func)
def test_has_pwm():
    '''
    Check detected PWM pins match expected pins.
    '''
    # Teensy has [34 digital IO pins][1].
    #
    # [1]: https://www.pjrc.com/teensy/teensy31.html
    pwm_pins = filter(lambda i: proxy.digital_pin_has_pwm(i), xrange(34))
    eq_(pwm_pins, [3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32])


@with_setup(setup_func, teardown_func)
def test_digital_write_read():
    '''
    Check digital write/read.
    '''
    proxy.pin_mode(13, 1)

    proxy.digital_write(13, 0)
    eq_(proxy.digital_read(13), 0)
    proxy.digital_write(13, 1)
    eq_(proxy.digital_read(13), 1)
    proxy.digital_write(13, 0)
    eq_(proxy.digital_read(13), 0)

    proxy.pin_mode(13, 0)
