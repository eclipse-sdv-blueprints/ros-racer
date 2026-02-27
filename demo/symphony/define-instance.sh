#!/bin/bash


# Check if JSON file argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <json-file>"
    echo "Example: $0 apriltag-tracking-instance.json"
    exit 1
fi

JSON_FILE="$1"

# Check if file exists
if [ ! -f "$JSON_FILE" ]; then
    echo "Error: File '$JSON_FILE' not found!"
    exit 1
fi

# Extract solution name from filename (remove path and .json extension)
ROOT_NAME=$(basename "$JSON_FILE" .json)
INSTANCE_NAME="${ROOT_NAME}"

export SYMPHONY_API_URL=http://localhost:8082/v1alpha2/

TOKEN=$(curl -X POST -H "Content-Type: application/json" -d '{"username":"admin","password":""}' "${SYMPHONY_API_URL}users/auth" | jq -r '.accessToken')


# Prompt user to press Enter to continue after the target has been registered

curl -X GET  -H "Content-Type: application/json"  -H "Authorization: Bearer $TOKEN"  "${SYMPHONY_API_URL}instances"

# Read & mutate JSON: overwrite metadata.name with INSTANCE_NAME using jq
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v jq >/dev/null 2>&1; then
        echo "Error: jq is required but not installed." >&2
        exit 2
fi

# Use --arg to safely inject shell variable
SOLUTION_DATA=$(jq --arg name "$INSTANCE_NAME" '(.metadata //= {}) | .metadata.name = $name' "$JSON_FILE")


HTTP_RESPONSE=$(curl -s -o /dev/null -w "%{http_code}"  -X POST \
    -H "Content-Type: application/json" \
    -H "Authorization: Bearer $TOKEN" \
    -d "$SOLUTION_DATA" \
    "${SYMPHONY_API_URL}instances/${INSTANCE_NAME}")
if [ "$HTTP_RESPONSE" -eq 200 ] || [ "$HTTP_RESPONSE" -eq 204 ]; then
    echo "Instance '${INSTANCE_NAME}' created successfully."
else
    echo "Failed to create instance '${INSTANCE_NAME}'. HTTP status: $HTTP_RESPONSE"
fi
