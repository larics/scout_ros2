#!/bin/bash

record_topics() {
  # record_topics output_directory topic1 [topic2 ... topicN]
  if [ "$#" -lt 2 ]; then
    echo "Usage: record_topics output_directory topic1 [topic2 ...]"
    return 1
  fi

  output_dir="$1"
  shift

  mkdir -p "$output_dir"

  for topic in "$@"; do
    # Sanitize topic name: remove leading slash and replace inner slashes with underscores.
    filename=$(echo "$topic" | sed 's|^/||; s|/|_|g')
    output_file="${output_dir}/${filename}.csv"

    echo "Recording topic '$topic' to '$output_file'..."
    ros2 topic echo --csv "$topic" 2>/dev/null > "$output_file" &
  done

  wait
}

# Call the function with the current topics.
record_topics "csv_data" \
  "/debug/debug/est/imu_baro_gps" \
  "/debug/debug/est/imu_baro_movella" \
  "/poses"
