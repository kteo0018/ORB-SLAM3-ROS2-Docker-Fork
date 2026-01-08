#!/bin/bash
set -euo pipefail

# Copy all files from the local results/ directory into this directory,
# renaming them to either orb_slam3_oriN or orb_slam3_afeatN (N starts at 1).

script_dir="$(cd "$(dirname "$0")" && pwd)"
results_dir="$script_dir/results"
dest_dir="$script_dir"

if [[ ! -d "$results_dir" ]]; then
    echo "Missing results directory: $results_dir" >&2
    exit 1
fi

# Choose mode via argument or interactive prompt
if [[ $# -ge 1 ]]; then
    mode="$1"
else
    echo "Choose copy mode:" >&2
    echo "  1) ori" >&2
    echo "  2) afeat" >&2
    read -rp "Enter 1 or 2: " choice
    case "$choice" in
        1) mode="ori" ;;
        2) mode="afeat" ;;
        *)
            echo "Invalid choice. Use 1 for ori or 2 for afeat." >&2
            exit 1
            ;;
    esac
fi

case "$mode" in
    ori)   base_name="orb_slam3_ori" ;;
    afeat) base_name="orb_slam3_afeat" ;;
    *)
        echo "Usage: $0 [ori|afeat]" >&2
        exit 1
        ;;
esac

mapfile -d '' files < <(find "$results_dir" -maxdepth 1 -type f -print0 | sort -z)

if [[ ${#files[@]} -eq 0 ]]; then
    echo "No files found in $results_dir" >&2
    exit 1
fi

counter=1
for f in "${files[@]}"; do
    filename="$(basename "$f")"
    ext="${filename##*.}"
    # Preserve extension if present
    if [[ "$ext" != "$filename" ]]; then
        new_name="${base_name}${counter}.${ext}"
    else
        new_name="${base_name}${counter}"
    fi

    cp "$f" "$dest_dir/$new_name"
    echo "Copied $filename -> $new_name"
    ((counter++))
done

echo "Done. Copied ${#files[@]} file(s) to $dest_dir using prefix '$base_name'."

