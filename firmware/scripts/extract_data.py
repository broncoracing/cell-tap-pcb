import sys

if_path = sys.argv[1]
of_path = if_path.replace('.log', '_new.csv')
with open(if_path, 'r') as infile:
    with open(of_path, 'w') as outfile:
        readings = [''] * 16
        for line in infile:
            stripped = line.replace('(', '').replace(') can0 ', '#')
            split = stripped.split('#')
            timestamp = float(split[0])
            arb_id = int(split[1], 16)
            board_id = (arb_id - 0x200) // 4
            message_no = arb_id % 4

            data = split[2].strip('\n')


            for i in range(4):
                reading_str = data[4 * i:4 * i + 4]

                readings[i + 4 * message_no] = str(float(int(reading_str, 16)) / 100.0)

            if all(reading != '' for reading in readings):
                out_str = f"{timestamp},{board_id},{','.join(readings)}\n"
                outfile.write(out_str)
                readings = [''] * 16
