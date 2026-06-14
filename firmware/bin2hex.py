#!/usr/bin/env python3
# bin2hex.py
# Converts a raw binary file to a 32-bit hex file for Verilog $readmemh.

import sys

def main():
    if len(sys.argv) < 3:
        print("Usage: bin2hex.py <input.bin> <output.hex>")
        sys.exit(1)

    in_path = sys.argv[1]
    out_path = sys.argv[2]

    with open(in_path, "rb") as f_in, open(out_path, "w") as f_out:
        words_written = 0
        while True:
            chunk = f_in.read(4)
            if not chunk:
                break
            
            # Pad chunk with zero bytes if not 4-byte aligned
            if len(chunk) < 4:
                chunk = chunk + b"\x00" * (4 - len(chunk))
                
            # Pack as a little-endian 32-bit word and format as big-endian hex string
            val = chunk[0] | (chunk[1] << 8) | (chunk[2] << 16) | (chunk[3] << 24)
            f_out.write(f"{val:08x}\n")
            words_written += 1
            
        # Pad the rest of the ROM (4096 words total) with standard NOP (0x00000013)
        while words_written < 4096:
            f_out.write("00000013\n")
            words_written += 1

    print(f"Successfully converted {in_path} to {out_path}")

if __name__ == "__main__":
    main()
