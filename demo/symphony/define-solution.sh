#!/bin/bash

# Script to create and deploy solution with base64 encoded stack data
# Compatible with Ubuntu, macOS, and WSL
# Usage: ./define-solution.sh <json-file>

# Function to check if a command exists
check_command() {
    if ! command -v "$1" &> /dev/null; then
        echo "Error: Required command '$1' not found. Please install it."
        exit 1
    fi
}

# Check required dependencies
check_command curl
check_command jq
check_command base64

# Check if JSON file argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <json-file>"
    echo "Example: $0 talker-listener-stack.json"
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
SOLUTION_NAME="${ROOT_NAME}-v-1"

export SYMPHONY_API_URL=http://localhost:8082/v1alpha2/

# Get authentication token
echo "Authenticating with Symphony API..."
TOKEN=$(curl -s -X POST -H "Content-Type: application/json" -d '{"username":"admin","password":""}' "${SYMPHONY_API_URL}users/auth" 2>/dev/null | jq -r '.accessToken' 2>/dev/null)

if [ -z "$TOKEN" ] || [ "$TOKEN" = "null" ]; then
    echo "Error: Failed to authenticate with Symphony API at $SYMPHONY_API_URL"
    echo "Please check that Symphony is running and accessible."
    exit 1
fi

# Function to encode base64 in a cross-platform way
encode_base64() {
    local file="$1"
    
    # Try different base64 approaches for maximum compatibility
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS (BSD base64)
        base64 -i "$file" 2>/dev/null || base64 < "$file" 2>/dev/null
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]] || [[ -n "$WSL_DISTRO_NAME" ]] || [[ -n "$IS_WSL" ]] || [[ "$(uname -r)" == *Microsoft* ]] || [[ "$(uname -r)" == *microsoft* ]]; then
        # WSL/Windows environments - try different approaches
        if command -v base64 >/dev/null 2>&1 && base64 -w 0 "$file" >/dev/null 2>&1; then
            base64 -w 0 "$file"
        elif command -v base64 >/dev/null 2>&1 && base64 < "$file" >/dev/null 2>&1; then
            base64 < "$file" | tr -d '\n'
        else
            # Fallback for older WSL versions
            cat "$file" | base64 | tr -d '\n'
        fi
    else
        # Linux/Unix (GNU base64)
        if command -v base64 >/dev/null 2>&1 && base64 -w 0 "$file" >/dev/null 2>&1; then
            base64 -w 0 "$file"
        else
            # Fallback for systems without -w option
            base64 < "$file" | tr -d '\n'
        fi
    fi
}

# Base64 encode the contents of the JSON file
echo "Encoding stack data to base64..."
STACK_DATA_BASE64=$(encode_base64 "$JSON_FILE")
if [ -z "$STACK_DATA_BASE64" ]; then
    echo "Error: Failed to base64 encode the JSON file"
    exit 1
fi

echo "Base64 encoded stack data length: ${#STACK_DATA_BASE64} characters"
# Create the solution JSON in a variable
SOLUTION_DATA=$(cat << EOF
{
    "metadata": {
        "namespace": "default",
        "name": "$SOLUTION_NAME"
    },
    "spec": {
        "displayName": "$SOLUTION_NAME",
        "rootResource": "$ROOT_NAME",
        "version": "1",
        "components": [
            {
                "name": "$SOLUTION_NAME",
                "type": "muto-agent",
                "properties": {
                    "type": "stack", 
                    "content-type": "application/json", 
                    "data": "$STACK_DATA_BASE64",
                    "foo": "bar",
                    "number": 123
                }
            }
        ]
    }
}
EOF
)

echo "Created solution JSON with base64 encoded stack data"
echo "Base64 encoded data length: ${#STACK_DATA_BASE64} characters"

# Show current solutions
# echo "Current solutions:"
# curl -s -X GET -H "Content-Type: application/json" -H "Authorization: Bearer $TOKEN" "${SYMPHONY_API_URL}solutions" | jq .

echo "Posting $SOLUTION_NAME solution to Symphony..."

# Try to delete existing solution first (ignore errors if it doesn't exist)
curl -s -X DELETE -H "Authorization: Bearer $TOKEN" "${SYMPHONY_API_URL}solutions/$SOLUTION_NAME" > /dev/null 2>&1

# Post the new solution
RESPONSE=$(curl -s -w "\n%{http_code}" -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $TOKEN" -d "$SOLUTION_DATA" "${SYMPHONY_API_URL}solutions/$SOLUTION_NAME" 2>/dev/null)

# Extract HTTP status code (last line) and response body (everything else)
HTTP_STATUS=$(echo "$RESPONSE" | tail -n1)
RESPONSE_BODY=$(echo "$RESPONSE" | head -n -1)

if [ "$HTTP_STATUS" -eq 200 ] || [ "$HTTP_STATUS" -eq 201 ]; then
    echo "✅ Solution '$SOLUTION_NAME' created successfully!"
    if [ -n "$RESPONSE_BODY" ] && [ "$RESPONSE_BODY" != "null" ]; then
        echo "Response: $RESPONSE_BODY"
    fi
else
    echo "❌ Failed to create solution. HTTP Status: $HTTP_STATUS"
    if [ -n "$RESPONSE_BODY" ]; then
        echo "Error details: $RESPONSE_BODY"
    fi
    exit 1
fi
