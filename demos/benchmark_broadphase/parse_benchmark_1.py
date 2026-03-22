import os
import re
import glob

def main():
    # Store the results as lists of tuples: (particles, time)
    cpu_results = []
    gpu_results =[]

    # Find all Benchmark 1 log files
    file_pattern = os.path.join("results", "bench1_particles_*.txt")
    files = glob.glob(file_pattern)

    if not files:
        print("No files found matching 'results/bench1_particles_*.txt'. Make sure you run this from the project root.")
        return

    for filepath in files:
        # Extract the particle count from the filename
        match = re.search(r'bench1_particles_(\d+)\.txt', filepath)
        if not match:
            continue
        
        particles = int(match.group(1))

        with open(filepath, 'r') as f:
            content = f.read()

        # Find the index of the *last* occurrence of the stats block
        last_block_idx = content.rfind("--- Profiler Stats ---")
        if last_block_idx == -1:
            print(f"Warning: No 'Profiler Stats' block found in {filepath}")
            continue

        # Slice the string to only look at the last block
        last_block = content[last_block_idx:]

        # Regex to find "Broad Phase" (CPU)
        cpu_match = re.search(r'^\s*Broad Phase\s*:\s*([\d.]+)\s*ms', last_block, re.MULTILINE)
        if cpu_match:
            cpu_results.append((particles, float(cpu_match.group(1))))

        # Regex to find the total time in the [cuda] log
        # We find all matches in the file and pick the last one
        gpu_matches = list(re.finditer(r'\[cuda\]\s+grid_b_half_shell.*?total=([\d.]+)\s*ms', content))
        if gpu_matches:
            gpu_results.append((particles, float(gpu_matches[-1].group(1))))
        else:
            print(f"Warning: No '[cuda] grid_b_half_shell' total time found in {filepath}")

    # Sort the results ascending by particle count
    cpu_results.sort(key=lambda x: x[0])
    gpu_results.sort(key=lambda x: x[0])

    # Write the formatted output to a file
    output_file = "parsed_benchmark1.txt"
    with open(output_file, 'w') as f:
        f.write("% CPU Results:\n")
        for p, time in cpu_results:
            f.write(f"({p}, {time:.4f})\n")
            
        # Keeping the exact label requested
        f.write("% GPU Grid A Results:\n")
        for p, time in gpu_results:
            f.write(f"({p}, {time:.4f})\n")

    print(f"Successfully parsed {len(files)} files!")
    print(f"Results saved to '{output_file}'.")

if __name__ == "__main__":
    main()
