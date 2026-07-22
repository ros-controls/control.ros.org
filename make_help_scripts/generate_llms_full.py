import os
import argparse

def stitch_text_file(source_dir, output_file):
    print(f"Stitching text files from {source_dir} into {output_file}...")
    output_dir = os.path.dirname(output_file)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(output_file, 'w', encoding = 'utf-8') as outfile:
        outfile.write("# ROS 2 Control Full Documentation\n\n")
        for root, dirs, files in os.walk(source_dir):
            dirs.sort()
            for file in sorted(files):
                if file.endswith('.txt'):
                    file_path = os.path.join(root, file)
                    outfile.write("\n" + "="*50 + "\n")
                    outfile.write(f"source page: {os.path.relpath(file_path, source_dir)}\n")
                    outfile.write("="*50 + "\n\n")

                    try:
                        with open(file_path, 'r', encoding = 'utf-8') as infile:
                            outfile.write(infile.read() + "\n")
                            outfile.write("\n")
                    except Exception as e:
                        print(f"Error reading file {file_path}: {e}")
        print("Stitching Complete")


if __name__ == '__main__':
    parser= argparse.ArgumentParser()
    parser.add_argument("--source", required= True, help = "Directory containing built .txt files")
    parser.add_argument("--output", required= True, help = "Path to llms-full.txt")
    args = parser.parse_args()

    stitch_text_file(args.source, args.output)
