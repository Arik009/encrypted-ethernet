print('memory_initialization_radix=16;')
print('memory_initialization_vector=')
NUM_ELEMENTS = 4096;
arr = list(range(NUM_ELEMENTS))
for i in range(NUM_ELEMENTS):
    print('%02x%c' % (arr[i] % 2**8, ',' if i != NUM_ELEMENTS - 1 else ';'))
