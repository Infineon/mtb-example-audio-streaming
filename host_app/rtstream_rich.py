# TODO get all this running under WSL2 => install this USB-port forwarding thing.

import time

# Use 'rich' package to provide pretty console output
from rich.console import Console
from rich.live import Live
from rich.table import Table

from host_app.rtstream_common import RTStreaming

rtstreaming = RTStreaming()
console = Console()

serial_stats = rtstreaming.get_serial_stats()
speech = rtstreaming.get_speech()


def gen_stats_table() -> Table:
    """Generate table with statistics."""
    table = Table()
    table.add_column("tstamp(ms)")
    table.add_column("avg tstamp delta(ms)")
    table.add_column("Max len COBS")
    table.add_column("0-len COBS")
    table.add_column("faulty 1st byte")

    table.add_row(
        f"{serial_stats.time_stamps[-1]}",
        f"{serial_stats.avg_delta_time_stamp:}",
        f"{serial_stats.max_len_cobs}",
        f"{serial_stats.num_zero_len_cobs}",
        f"0x{serial_stats.faulty_first_byte:02x}"
    )

    return table


rtstreaming.setup_serial_connection()

# running count of COBS packets with wrong length
len_failure_cnt = 0

console.print("[green]Streaming speech data ...[/green]")
cur_time = time.time()

with Live(gen_stats_table()) as live:
    while 1:
        ret = rtstreaming.run_streaming()

        if ret != 0:
            len_failure_cnt += 1
            print(f"error: incomplete packet, total num len failures: {len_failure_cnt}")

            if len_failure_cnt > 4:
                print("more than 4 bad blobs received: stop streaming")
                exit()

        live.update(gen_stats_table())
