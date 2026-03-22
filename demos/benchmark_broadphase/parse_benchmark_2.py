import os
import re
import glob

def main():
    # Dictionary to hold the results
    # format: results[strategy]['build'] = [(res, time), ...]
    results = {
        'A': {'build':[], 'query': []},
        'B_NAIVE': {'build': [], 'query':[]},
        'B_HALF_SHELL': {'build': [], 'query':[]}
    }

    # Find all Benchmark 2 log files
    file_pattern = os.path.join("results", "bench2_res_*.txt")
    files = glob.glob(file_pattern)

    if not files:
        print("No files found matching 'results/bench2_res_*.txt'. Make sure you run this from the project root.")
        return

    for filepath in files:
        basename = os.path.basename(filepath)
        
        # Extract the resolution and strategy from the filename
        # e.g., bench2_res_100_B_HALF_SHELL.txt -> res: 100, strategy: B_HALF_SHELL
        match = re.search(r'bench2_res_(\d+)_(A|B_NAIVE|B_HALF_SHELL)\.txt', basename)
        if not match:
            continue
        
        res = int(match.group(1))
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
            
        # Take the very last occurrence in the file
        last_match = matches[-1]
        build_time = float(last_match.group(1))
        query_time = float(last_match.group(2))

        results[strategy]['build'].append((res, build_time))
        results[strategy]['query'].append((res, query_time))

    # Output formatting
    output_file = "parsed_benchmark2.txt"
    with open(output_file, 'w') as f:
        for strategy in ['A', 'B_NAIVE', 'B_HALF_SHELL']:
            # Sort descending by resolution (100, 80, 60, 40, 20)
            results[strategy]['build'].sort(key=lambda x: x[0], reverse=True)
            results[strategy]['query'].sort(key=lambda x: x[0], reverse=True)

            # Format Build Line
            build_str = " ".join([f"({r}, {t:.3f})" for r, t in results[strategy]['build']])
            f.write(f"% Strategy {strategy} - Build Phase:\n")
            f.write(f"{build_str}\n")

            # Format Query Line
            query_str = " ".join([f"({r}, {t:.3f})" for r, t in results[strategy]['query']])
            f.write(f"% Strategy {strategy} - Query Phase:\n")
            f.write(f"{query_str}\n\n")

    print(f"Successfully parsed {len(files)} files!")
    print(f"Results saved to '{output_file}'.")

if __name__ == "__main__":
    main()
