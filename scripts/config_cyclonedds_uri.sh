#!/bin/bash

# Copy configuration file to system location (requires sudo)
copy_config() {
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local system_xml="/etc/cyclonedds/cyclonedds.xml"

    echo "Copying cyclonedds.xml to system location..."
    sudo mkdir -p /etc/cyclonedds
    sudo cp "$script_dir/cyclonedds.xml" "$system_xml"

    # Make the file readable by all users
    sudo chmod 644 "$system_xml"
    echo "✓ Copied to $system_xml (readable by all users)"
}

# Add to current user's bashrc
add_to_bashrc() {
    local export_line='export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds.xml"'
    local bashrc_file="$HOME/.bashrc"

    if grep -q "CYCLONEDDS_URI=" "$bashrc_file"; then
        sed -i "s|export CYCLONEDDS_URI=.*|$export_line|" "$bashrc_file"
        echo "✓ Updated CYCLONEDDS_URI in ~/.bashrc"
    else
        {
            echo ""
            echo "# CycloneDDS configuration"
            echo "$export_line"
        } >> "$bashrc_file"
        echo "✓ Added CYCLONEDDS_URI to ~/.bashrc"
    fi
}

# Main execution
main() {
    copy_config
    add_to_bashrc

    echo ""
    echo "Configuration completed!"
    echo ""
    echo "Run 'source ~/.bashrc' or open a new terminal to apply."
    echo ""
    echo "Note: Other users need to add this line to their ~/.bashrc:"
    echo "  export CYCLONEDDS_URI=\"file:///etc/cyclonedds/cyclonedds.xml\""
}

main "$@"