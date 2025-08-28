"""
Statistical Analysis and Report Generation

Processes statistical validation results and generates formatted output
suitable for academic papers, including LaTeX-ready tables and confidence intervals.
"""

import json
import numpy as np
import pandas as pd
from typing import Dict, List, Tuple, Optional
import scipy.stats as stats
from datetime import datetime
import matplotlib.pyplot as plt
import seaborn as sns

class StatisticalAnalyzer:
    """
    Analyze and format statistical validation results for academic presentation
    """
    
    def __init__(self, results_file: str):
        """Load statistical validation results from JSON file"""
        with open(results_file, 'r') as f:
            self.results = json.load(f)
        
        self.method_stats = self.results.get('method_statistics', {})
        self.correlations = self.results.get('correlation_analysis', {})
        self.config = self.results.get('configuration', {})
    
    def format_confidence_interval(self, mean: float, ci_tuple: Tuple[float, float], 
                                 precision: int = 1) -> str:
        """
        Format confidence interval in academic style: μ = X.X ± Y.Y (95% CI)
        """
        ci_lower, ci_upper = ci_tuple
        margin_of_error = ci_upper - mean
        return f"{mean:.{precision}f} $\\pm$ {margin_of_error:.{precision}f}"
    
    def generate_processing_time_table(self) -> str:
        """
        Generate LaTeX table for processing time results
        """
        latex_lines = [
            "\\begin{table}[htbp]",
            "\\centering",
            "\\caption{Processing Time Analysis by Detection Method}",
            "\\begin{tabular}{lcccc}",
            "\\toprule",
            "Method & Mean (ms) & 95\\% CI (ms) & Sample Size & Std Dev \\\\",
            "\\midrule"
        ]
        
        for method, stats in self.method_stats.items():
            if 'processing_time' in stats:
                pt = stats['processing_time']
                mean_ms = pt['mean_ms']
                ci = pt['confidence_interval_ms']
                n = pt['sample_size']
                std = pt['std_ms']
                
                # Format method name
                method_display = method.replace('_', ' ').title()
                
                # Format confidence interval
                ci_str = self.format_confidence_interval(mean_ms, ci)
                
                line = f"{method_display} & {mean_ms:.1f} & {ci_str} & {n} & {std:.1f} \\\\"
                latex_lines.append(line)
        
        latex_lines.extend([
            "\\bottomrule",
            "\\end{tabular}",
            "\\label{tab:processing_times}",
            "\\end{table}"
        ])
        
        return '\n'.join(latex_lines)
    
    def generate_detection_count_table(self) -> str:
        """
        Generate LaTeX table for detection count results
        """
        latex_lines = [
            "\\begin{table}[htbp]",
            "\\centering",
            "\\caption{Target Detection Count Analysis by Method}",
            "\\begin{tabular}{lcccc}",
            "\\toprule",
            "Method & Mean Count & 95\\% CI & Sample Size & Std Dev \\\\",
            "\\midrule"
        ]
        
        for method, stats in self.method_stats.items():
            if 'detection_count' in stats:
                dc = stats['detection_count']
                mean_count = dc['mean']
                ci = dc['confidence_interval']
                n = dc['sample_size']
                std = dc['std']
                
                method_display = method.replace('_', ' ').title()
                ci_str = self.format_confidence_interval(mean_count, ci)
                
                line = f"{method_display} & {mean_count:.1f} & {ci_str} & {n} & {std:.1f} \\\\"
                latex_lines.append(line)
        
        latex_lines.extend([
            "\\bottomrule",
            "\\end{tabular}",
            "\\label{tab:detection_counts}",
            "\\end{table}"
        ])
        
        return '\n'.join(latex_lines)
    
    def generate_correlation_table(self) -> str:
        """
        Generate LaTeX table for correlation analysis
        """
        if not self.correlations:
            return "% No correlation data available"
        
        latex_lines = [
            "\\begin{table}[htbp]",
            "\\centering",
            "\\caption{Environmental Factor Correlation Analysis}",
            "\\begin{tabular}{lcc}",
            "\\toprule",
            "Correlation & Coefficient (r) & Interpretation \\\\",
            "\\midrule"
        ]
        
        for corr_name, corr_value in self.correlations.items():
            # Format correlation name
            display_name = corr_name.replace('_', ' ').title()
            
            # Interpret correlation strength
            abs_r = abs(corr_value)
            if abs_r >= 0.8:
                interpretation = "Strong"
            elif abs_r >= 0.5:
                interpretation = "Moderate" 
            elif abs_r >= 0.3:
                interpretation = "Weak"
            else:
                interpretation = "Negligible"
            
            line = f"{display_name} & {corr_value:.3f} & {interpretation} \\\\"
            latex_lines.append(line)
        
        latex_lines.extend([
            "\\bottomrule",
            "\\end{tabular}",
            "\\label{tab:correlations}",
            "\\end{table}"
        ])
        
        return '\n'.join(latex_lines)
    
    def generate_text_summary(self) -> str:
        """
        Generate text summary suitable for paper's results section
        """
        lines = []
        lines.append("STATISTICAL VALIDATION RESULTS")
        lines.append("=" * 50)
        lines.append("")
        
        # Configuration summary
        successful_runs = self.config.get('successful_runs', 0)
        total_runs = self.config.get('num_runs', 0)
        confidence_level = self.config.get('confidence_level', 0.95)
        
        lines.append(f"Monte Carlo validation employed {total_runs} randomized scenarios with {successful_runs} successful executions, providing {confidence_level*100:.0f}% confidence intervals through bootstrap resampling.")
        lines.append("")
        
        # Processing time results
        lines.append("Processing Time Analysis:")
        for method, stats in self.method_stats.items():
            if 'processing_time' in stats:
                pt = stats['processing_time']
                mean_ms = pt['mean_ms']
                ci = pt['confidence_interval_ms']
                n = pt['sample_size']
                
                method_display = method.replace('_', ' ').capitalize()
                ci_str = self.format_confidence_interval(mean_ms, ci)
                
                lines.append(f"- {method_display}: μ = {ci_str} ms (95% CI, n={n})")
        
        lines.append("")
        
        # Detection count results
        lines.append("Detection Count Analysis:")
        for method, stats in self.method_stats.items():
            if 'detection_count' in stats:
                dc = stats['detection_count']
                mean_count = dc['mean']
                ci = dc['confidence_interval']
                n = dc['sample_size']
                
                method_display = method.replace('_', ' ').capitalize()
                ci_str = self.format_confidence_interval(mean_count, ci)
                
                lines.append(f"- {method_display}: μ = {ci_str} targets (95% CI, n={n})")
        
        # Statistical comparisons
        if len(self.method_stats) >= 2:
            lines.append("")
            lines.append("Statistical Comparisons:")
            lines.append(self._generate_statistical_comparisons())
        
        # Environmental correlations
        if self.correlations:
            lines.append("")
            lines.append("Environmental Correlation Analysis:")
            for corr_name, corr_value in self.correlations.items():
                display_name = corr_name.replace('_', ' ')
                lines.append(f"- {display_name}: r = {corr_value:.3f}")
        
        return '\n'.join(lines)
    
    def _generate_statistical_comparisons(self) -> str:
        """
        Generate statistical comparison text between methods
        """
        comparisons = []
        
        # Get processing times for comparison
        processing_times = {}
        detection_counts = {}
        
        for method, stats in self.method_stats.items():
            if 'processing_time' in stats:
                pt = stats['processing_time']
                processing_times[method] = {
                    'mean': pt['mean_ms'],
                    'ci': pt['confidence_interval_ms']
                }
            
            if 'detection_count' in stats:
                dc = stats['detection_count']
                detection_counts[method] = {
                    'mean': dc['mean'],
                    'ci': dc['confidence_interval']
                }
        
        # Processing time comparisons
        if len(processing_times) >= 2:
            methods = list(processing_times.keys())
            fastest_method = min(methods, key=lambda m: processing_times[m]['mean'])
            slowest_method = max(methods, key=lambda m: processing_times[m]['mean'])
            
            fastest_time = processing_times[fastest_method]['mean']
            slowest_time = processing_times[slowest_method]['mean']
            
            if fastest_method != slowest_method:
                speedup = slowest_time / fastest_time
                comparisons.append(
                    f"- {fastest_method.replace('_', ' ').capitalize()} demonstrates {speedup:.1f}x faster processing than {slowest_method.replace('_', ' ')}"
                )
        
        # Detection count comparisons
        if len(detection_counts) >= 2:
            methods = list(detection_counts.keys())
            best_method = max(methods, key=lambda m: detection_counts[m]['mean'])
            worst_method = min(methods, key=lambda m: detection_counts[m]['mean'])
            
            if best_method != worst_method:
                best_count = detection_counts[best_method]['mean']
                worst_count = detection_counts[worst_method]['mean']
                improvement = (best_count - worst_count) / worst_count * 100
                
                comparisons.append(
                    f"- {best_method.replace('_', ' ').capitalize()} detects {improvement:.0f}% more targets than {worst_method.replace('_', ' ')}"
                )
        
        return '\n'.join(comparisons) if comparisons else "No significant differences detected."
    
    def perform_significance_tests(self) -> Dict[str, float]:
        """
        Perform statistical significance tests between methods
        Note: This is a simplified approach - in practice you'd need the raw data
        """
        significance_results = {}
        
        # This would require access to raw data for proper t-tests
        # For now, we estimate based on confidence intervals
        methods = list(self.method_stats.keys())
        
        if len(methods) >= 2:
            for i, method1 in enumerate(methods):
                for j in range(i + 1, len(methods)):
                    method2 = methods[j]
                    
                    # Compare processing times if available
                    if ('processing_time' in self.method_stats[method1] and 
                        'processing_time' in self.method_stats[method2]):
                        
                        pt1 = self.method_stats[method1]['processing_time']
                        pt2 = self.method_stats[method2]['processing_time']
                        
                        # Check if confidence intervals overlap
                        ci1 = pt1['confidence_interval_ms']
                        ci2 = pt2['confidence_interval_ms']
                        
                        # If CIs don't overlap, difference is likely significant
                        no_overlap = (ci1[1] < ci2[0]) or (ci2[1] < ci1[0])
                        
                        comparison_name = f"{method1}_vs_{method2}_processing_time"
                        significance_results[comparison_name] = {
                            'likely_significant': no_overlap,
                            'method1_ci': ci1,
                            'method2_ci': ci2
                        }
        
        return significance_results
    
    def generate_complete_report(self, output_file: str = None) -> str:
        """
        Generate complete statistical analysis report
        """
        report_lines = []
        
        # Header
        report_lines.extend([
            "STATISTICAL VALIDATION REPORT",
            "Generated on: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "=" * 80,
            ""
        ])
        
        # Configuration
        report_lines.extend([
            "VALIDATION CONFIGURATION:",
            f"- Total Monte Carlo runs: {self.config.get('num_runs', 'N/A')}",
            f"- Successful runs: {self.config.get('successful_runs', 'N/A')}",
            f"- Confidence level: {self.config.get('confidence_level', 0.95)*100:.0f}%",
            f"- Parallel execution: {self.config.get('parallel_execution', 'N/A')}",
            ""
        ])
        
        # Text summary
        report_lines.append(self.generate_text_summary())
        report_lines.append("")
        
        # LaTeX tables
        report_lines.extend([
            "LATEX TABLES FOR ACADEMIC PUBLICATION:",
            "=" * 50,
            "",
            "Processing Time Table:",
            self.generate_processing_time_table(),
            "",
            "Detection Count Table:",
            self.generate_detection_count_table(),
            "",
            "Correlation Analysis Table:",
            self.generate_correlation_table(),
            ""
        ])
        
        # Significance tests
        significance_results = self.perform_significance_tests()
        if significance_results:
            report_lines.extend([
                "STATISTICAL SIGNIFICANCE ANALYSIS:",
                "=" * 50
            ])
            for test_name, result in significance_results.items():
                report_lines.append(f"{test_name}: {'Likely significant' if result['likely_significant'] else 'Not significant'}")
        
        complete_report = '\n'.join(report_lines)
        
        # Save to file if requested
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(complete_report)
            print(f"Complete report saved to: {output_file}")
        
        return complete_report
    
    def create_visualization_plots(self, output_dir: str = "statistical_plots"):
        """
        Create visualization plots for the statistical results
        """
        import os
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Processing time comparison plot
        plt.figure(figsize=(10, 6))
        
        methods = []
        means = []
        errors = []
        
        for method, stats in self.method_stats.items():
            if 'processing_time' in stats:
                pt = stats['processing_time']
                methods.append(method.replace('_', ' ').title())
                means.append(pt['mean_ms'])
                # Error bars represent confidence interval width
                ci_lower, ci_upper = pt['confidence_interval_ms']
                errors.append(ci_upper - pt['mean_ms'])
        
        if methods:
            plt.bar(methods, means, yerr=errors, capsize=5, alpha=0.7)
            plt.ylabel('Processing Time (ms)')
            plt.title('Processing Time Comparison with 95% Confidence Intervals')
            plt.xticks(rotation=45, ha='right')
            plt.tight_layout()
            plt.savefig(f'{output_dir}/processing_time_comparison.png', dpi=150, bbox_inches='tight')
            plt.close()
        
        # Detection count comparison plot
        plt.figure(figsize=(10, 6))
        
        methods = []
        means = []
        errors = []
        
        for method, stats in self.method_stats.items():
            if 'detection_count' in stats:
                dc = stats['detection_count']
                methods.append(method.replace('_', ' ').title())
                means.append(dc['mean'])
                ci_lower, ci_upper = dc['confidence_interval']
                errors.append(ci_upper - dc['mean'])
        
        if methods:
            plt.bar(methods, means, yerr=errors, capsize=5, alpha=0.7, color='orange')
            plt.ylabel('Average Detection Count')
            plt.title('Detection Count Comparison with 95% Confidence Intervals')
            plt.xticks(rotation=45, ha='right')
            plt.tight_layout()
            plt.savefig(f'{output_dir}/detection_count_comparison.png', dpi=150, bbox_inches='tight')
            plt.close()
        
        print(f"Visualization plots saved to '{output_dir}/' directory")

def main():
    """
    Main function for analyzing statistical validation results
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='Analyze Statistical Validation Results')
    parser.add_argument('results_file', help='Path to statistical validation results JSON file')
    parser.add_argument('--output', '-o', help='Output file for complete report')
    parser.add_argument('--latex-only', action='store_true', help='Only generate LaTeX tables')
    parser.add_argument('--plots', action='store_true', help='Generate visualization plots')
    
    args = parser.parse_args()
    
    try:
        # Load and analyze results
        analyzer = StatisticalAnalyzer(args.results_file)
        
        if args.latex_only:
            # Generate LaTeX tables only
            print("PROCESSING TIME TABLE:")
            print(analyzer.generate_processing_time_table())
            print("\nDETECTION COUNT TABLE:")
            print(analyzer.generate_detection_count_table())
            print("\nCORRELATION TABLE:")
            print(analyzer.generate_correlation_table())
        else:
            # Generate complete report
            output_file = args.output or f"statistical_analysis_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            report = analyzer.generate_complete_report(output_file)
            print("STATISTICAL ANALYSIS SUMMARY:")
            print("=" * 50)
            print(analyzer.generate_text_summary())
        
        # Generate plots if requested
        if args.plots:
            analyzer.create_visualization_plots()
    
    except FileNotFoundError:
        print(f"Error: Results file '{args.results_file}' not found")
    except Exception as e:
        print(f"Error analyzing results: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
        