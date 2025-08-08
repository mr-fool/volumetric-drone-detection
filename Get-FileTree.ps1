# PowerShell script to generate a file tree structure excluding node_modules
# Save this as Get-FileTree.ps1

param(
    [Parameter(Mandatory=$false)]
    [string]$Path = ".",
    
    [Parameter(Mandatory=$false)]
    [string]$OutputFile = "file_tree.txt",
    
    [Parameter(Mandatory=$false)]
    [string[]]$ExcludeFolders = @("node_modules", ".git", "dist", "build"),
    
    [Parameter(Mandatory=$false)]
    [int]$MaxDepth = -1
)

function Get-Tree {
    param(
        [string]$Path,
        [string]$Indent = "",
        [int]$CurrentDepth = 0
    )
    
    # Check if we've reached max depth
    if ($MaxDepth -ne -1 -and $CurrentDepth -gt $MaxDepth) {
        return
    }

    # Get all items in the current directory
    $items = Get-ChildItem -Path $Path -Force
    
    # Process each item
    foreach ($item in $items) {
        # Output the current item
        "$Indent|-- $($item.Name)"
        
        # If it's a directory and not in the exclude list, process its contents
        if ($item.PSIsContainer -and $ExcludeFolders -notcontains $item.Name) {
            Get-Tree -Path $item.FullName -Indent "$Indent|   " -CurrentDepth ($CurrentDepth + 1)
        }
    }
}

# Get the absolute path
$AbsolutePath = Resolve-Path $Path

# Start the output with the root directory
"Directory Tree for: $AbsolutePath (excluding: $($ExcludeFolders -join ', '))" | Out-File -FilePath $OutputFile
"" | Out-File -FilePath $OutputFile -Append
$rootDirName = Split-Path $AbsolutePath -Leaf
"$rootDirName" | Out-File -FilePath $OutputFile -Append

# Generate the tree and append to the output file
Get-Tree -Path $AbsolutePath | Out-File -FilePath $OutputFile -Append

Write-Host "File tree has been generated and saved to $OutputFile"