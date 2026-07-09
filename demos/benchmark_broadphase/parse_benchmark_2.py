import os
import re
import glob

def main():
    results = {
        'A': {'build':[], 'query':[]},
        'B_NAIVE': {'build': [], 'query':[]},
        'B_HALF_SHELL': {'build': [], 'query':[]}
    }

    # Find all Benchmark 2 log files
    file_pattern = os.path.join("bench_broad_results", "bench2_cell_*.txt")
    files = glob.glob(file_pattern)

    if not files:
        print("No files found matching 'bench_broad_results/bench2_cell_*.txt'")
        return

    for filepath in files:
        basename = os.path.basename(filepath)
        
        # Extract the cell size and strategy from the filename
        match = re.search(r'bench2_cell_(\d+)_(A|B_NAIVE|B_HALF_SHELL)\.txt', basename)
        if not match:
            continue

        cell_size = int(match.group(1))
        strategy = match.group(2)

        with open(filepath, 'r') as f:
            content = f.read()

        # Regex to find the [cuda] log line that contains build=...ms and query=...ms
        # We find all matches in the file and pick the last one.
        pattern = r'\[cuda\]\s+.*?[bB]uild=([\d.]+)\s*ms.*?[qQ]uery=([\d.]+)\s*ms'
        matches = list(re.finditer(pattern, content))
        
        if not matches:
            print(f"Warning: Could not find [cuda] build/query metrics in {filepath}")
            continue
            
        last_match = matches[-1]
        build_time = float(last_match.group(1))
        query_time = float(last_match.group(2))

        # Append using cell_size as the X value
        results[strategy]['build'].append((cell_size, build_time))
        results[strategy]['query'].append((cell_size, query_time))

    output_file = "bench_broad_results/parsed_benchmark2.txt"
    with open(output_file, 'w') as f:
        for strategy in ['A', 'B_NAIVE', 'B_HALF_SHELL']:
            # Sort ascending by Cell Size
            results[strategy]['build'].sort(key=lambda x: x[0])
            results[strategy]['query'].sort(key=lambda x: x[0])

            build_str = " ".join([f"({c}, {t:.3f})" for c, t in results[strategy]['build']])
            f.write(f"% Strategy {strategy} - Build Phase:\n")
            f.write(f"{build_str}\n")

            query_str = " ".join([f"({c}, {t:.3f})" for c, t in results[strategy]['query']])
            f.write(f"% Strategy {strategy} - Query Phase:\n")
            f.write(f"{query_str}\n\n")

    print(f"Successfully parsed {len(files)} files!")
    print(f"Results saved to '{output_file}'.")

if __name__ == "__main__":
    main()
