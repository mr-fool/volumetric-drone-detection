import matplotlib.pyplot as plt
import numpy as np

def generate_memory_vs_resolution_plot():
    """
    Generate a professional memory vs voxel resolution plot for academic paper
    Based on data from your README specifications
    """
    
    # Data from your README - Voxel Resolution Impact table
    resolutions = np.array([10.0, 7.0, 5.0, 3.0, 2.0, 1.5])  # meters
    voxel_counts = np.array([450000, 1.3e6, 3.6e6, 16.7e6, 56.3e6, 133e6])  # total voxels
    memory_mb = np.array([10, 30, 82, 381, 1300, 3000])  # MB
    
    # Performance categories from your data
    performance_labels = ['Fastest', 'Fast', 'Moderate', 'Slow', 'Very Slow', 'Impractical']
    colors = ['#2E8B57', '#32CD32', '#FFD700', '#FF8C00', '#FF4500', '#DC143C']
    
    # Create figure with academic styling
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Voxel Resolution Impact on System Resources', fontsize=14, fontweight='bold')
    
    # Top plot: Memory Usage
    ax1.loglog(resolutions, memory_mb, 'o-', linewidth=2, markersize=8, color='#1f77b4', label='Memory Usage')
    
    # Color-code points by performance category
    for i, (res, mem, color, perf) in enumerate(zip(resolutions, memory_mb, colors, performance_labels)):
        ax1.scatter(res, mem, c=color, s=100, zorder=5, edgecolors='black', linewidth=1)
        
        # Add performance labels
        if i < 4:  # Only label practical configurations
            ax1.annotate(f'{perf}\n({mem} MB)', 
                        xy=(res, mem), 
                        xytext=(10, 10), 
                        textcoords='offset points',
                        fontsize=9,
                        ha='left',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor=color, alpha=0.7))
    
    ax1.set_xlabel('Voxel Resolution ρ (meters)', fontsize=12)
    ax1.set_ylabel('Memory Usage (MB)', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Memory Scaling with Voxel Resolution', fontsize=12, fontweight='bold')
    
    # Add practical usage zones
    ax1.axvspan(5.0, 10.0, alpha=0.2, color='green', label='Recommended Range')
    ax1.axvspan(2.0, 5.0, alpha=0.2, color='orange', label='High-Precision Range')
    ax1.axvspan(1.0, 2.0, alpha=0.2, color='red', label='Impractical Range')
    
    ax1.legend(loc='upper right')
    
    # Bottom plot: Computational Complexity
    voxel_counts_millions = voxel_counts / 1e6
    
    ax2.loglog(resolutions, voxel_counts_millions, 's-', linewidth=2, markersize=8, color='#ff7f0e', label='Total Voxels')
    
    # Color-code points
    for i, (res, voxels, color, perf) in enumerate(zip(resolutions, voxel_counts_millions, colors, performance_labels)):
        ax2.scatter(res, voxels, c=color, s=100, zorder=5, edgecolors='black', linewidth=1)
    
    ax2.set_xlabel('Voxel Resolution ρ (meters)', fontsize=12)
    ax2.set_ylabel('Total Voxels (millions)', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Computational Complexity Scaling', fontsize=12, fontweight='bold')
    
    # Add theoretical scaling line
    res_theory = np.linspace(1.5, 10.0, 100)
    # Theoretical: Total_Voxels = (1000/ρ) × (1000/ρ) × (450/ρ) for your 1km×1km×450m volume
    voxels_theory = (1000 * 1000 * 450) / (res_theory**3) / 1e6
    ax2.plot(res_theory, voxels_theory, '--', color='gray', alpha=0.7, label='Theoretical O(ρ⁻³)')
    
    ax2.legend(loc='upper right')
    
    # Add usage recommendations text box
    textstr = '\n'.join([
        'Configuration Guidelines:',
        '• Development/Testing: ρ = 10.0m',
        '• Standard Operation: ρ = 5.0-7.0m', 
        '• High-Precision: ρ = 3.0m',
        '• Research Maximum: ρ = 2.0m'
    ])
    
    props = dict(boxstyle='round', facecolor='lightblue', alpha=0.8)
    fig.text(0.02, 0.02, textstr, fontsize=10, verticalalignment='bottom', bbox=props)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.92, bottom=0.15)
    
    # Save with high DPI for publication
    plt.savefig('memory_vs_voxel_resolution.png', dpi=300, bbox_inches='tight')
    plt.savefig('memory_vs_voxel_resolution.pdf', bbox_inches='tight')  # For LaTeX
    
    plt.show()
    
    print("Generated publication-quality plots:")
    print("- memory_vs_voxel_resolution.png (300 DPI)")
    print("- memory_vs_voxel_resolution.pdf (vector format)")

def generate_simplified_version():
    """
    Generate a simplified single-plot version for space-constrained papers
    """
    resolutions = np.array([10.0, 7.0, 5.0, 3.0, 2.0, 1.5])
    memory_mb = np.array([10, 30, 82, 381, 1300, 3000])
    performance_categories = ['Development', 'Standard', 'Standard', 'High-Precision', 'Research', 'Impractical']
    colors = ['#2E8B57', '#32CD32', '#32CD32', '#FF8C00', '#FF4500', '#DC143C']
    
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    
    # Main curve
    ax.loglog(resolutions, memory_mb, 'o-', linewidth=2, markersize=10, color='#1f77b4')
    
    # Color-coded points with labels
    for i, (res, mem, color, cat) in enumerate(zip(resolutions, memory_mb, colors, performance_categories)):
        ax.scatter(res, mem, c=color, s=120, zorder=5, edgecolors='black', linewidth=1.5)
        
        if i < 4:  # Label practical points only
            ax.annotate(f'ρ={res}m\n{mem} MB', 
                       xy=(res, mem), 
                       xytext=(15, 15), 
                       textcoords='offset points',
                       fontsize=10,
                       ha='left',
                       bbox=dict(boxstyle='round,pad=0.4', facecolor=color, alpha=0.8))
    
    # Add mathematical relationship
    res_fit = np.linspace(1.5, 10.0, 100)
    # From your formula: Memory ∝ (Area/ρ)³
    memory_fit = 450000 * (10.0/res_fit)**3 * 3 * 8 / (1024**2)  # Theoretical curve
    ax.plot(res_fit, memory_fit, '--', color='gray', alpha=0.7, linewidth=2, 
            label='Theoretical: Memory ∝ ρ⁻³')
    
    ax.set_xlabel('Voxel Resolution ρ (meters)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Memory Usage (MB)', fontsize=14, fontweight='bold')
    ax.set_title('Memory Scaling with Voxel Resolution\n(1 km² surveillance area)', 
                fontsize=14, fontweight='bold')
    
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12)
    
    # Add usage zones
    ax.axvspan(5.0, 10.0, alpha=0.15, color='green', label='Recommended')
    ax.axvspan(3.0, 5.0, alpha=0.15, color='orange', label='High-Precision')
    
    plt.tight_layout()
    plt.savefig('figure_memory_scaling.png', dpi=300, bbox_inches='tight')
    plt.savefig('figure_memory_scaling.pdf', bbox_inches='tight')
    plt.show()
    
    print("Generated simplified figure for paper inclusion:")
    print("- figure_memory_scaling.png")
    print("- figure_memory_scaling.pdf")

if __name__ == "__main__":
    print("Generating memory vs voxel resolution plots...")
    print("\n1. Full analysis version:")
    generate_memory_vs_resolution_plot()
    
    print("\n2. Simplified paper version:")
    generate_simplified_version()
    
    print("\nPlots generated based on your README specifications:")
    print("- Uses actual data from your voxel resolution impact table")
    print("- Shows memory scaling relationship Memory ∝ ρ⁻³") 
    print("- Includes practical configuration recommendations")
    print("- Publication-ready formatting at 300 DPI")