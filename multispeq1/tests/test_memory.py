import nose.tools as nt
import numpy as np
import multispeq1 as tr


def setup_func():
    global proxy
    proxy = tr.SerialProxy()


def teardown_func():
    global proxy
    del proxy


@nt.with_setup(setup_func, teardown_func)
def test_get_buffer():
    '''
    Test retrieving node buffer contents.
    '''
    proxy.get_buffer()


@nt.with_setup(setup_func, teardown_func)
def test_malloc_free():
    '''
    Test dynamic memory allocation on Teensy.
    '''
    start_ram_free = proxy.ram_free()

    # Allocate 1024 bytes on Teensy device.
    data_addr = proxy.mem_alloc(1 << 10)
    # Verify at least the amount of requested memory has been allocated.
    nt.assert_greater_equal(start_ram_free - proxy.ram_free(), 1 << 10)
    proxy.mem_free(data_addr)
    # Verify memory has been freed.
    nt.eq_(proxy.ram_free(), start_ram_free)


@nt.with_setup(setup_func, teardown_func)
def test_mem_copy():
    '''
    Test copying between host memory and device memory.
    '''
    for N_i in (16, 32, 1024, 2048):
        yield check_mem_copy, N_i


def check_mem_copy(N):
    # Select CHUNK_SIZE as a multiple of 2, allow up to 16 bytes for packet
    # header.
    CHUNK_SIZE = 2 * (proxy.max_serial_payload_size() // 2) - 16

    data_addr = proxy.mem_alloc(2 * N)
    try:
        data = np.arange(N, dtype='uint16').view('uint8')
        for i in xrange(0, int(np.ceil(2 * N / float(CHUNK_SIZE)))):
            start_i = CHUNK_SIZE * i
            end_i = min(2 * N, CHUNK_SIZE * (i + 1))
            proxy.mem_cpy_host_to_device(data_addr + (CHUNK_SIZE * i),
                                         data[start_i:end_i])
        device_data = proxy.mem_cpy_device_to_host(data_addr, 2 *
                                                   N).view('uint16')
        np.testing.assert_array_equal(data.view('uint16'), device_data)
    finally:
        proxy.mem_free(data_addr)


@nt.with_setup(setup_func, teardown_func)
def test_mem_fill():
    for value_i in (np.uint8(123), np.uint16(1234), np.uint32(876543),
                    np.float32(98.765)):
        yield check_mem_fill, value_i


def check_mem_fill(value):
    '''
    Test filling device memory with scalar value.
    '''
    itemsize = value.dtype.itemsize
    # Allocate memory on device.
    data_addr = proxy.mem_alloc(512 * itemsize)

    try:
        # Fill device memory with value 1234.
        fill_type = (value.dtype.name
                     if not 'float' in value.dtype.name else 'float')
        f_mem_fill = getattr(proxy, 'mem_fill_' + fill_type)
        f_mem_fill(data_addr, value, 512)
        device_data = proxy.mem_cpy_device_to_host(data_addr, 512 *
                                                   itemsize).view(value.dtype)

        np.testing.assert_array_equal(device_data, value)
    finally:
        proxy.mem_free(data_addr)
