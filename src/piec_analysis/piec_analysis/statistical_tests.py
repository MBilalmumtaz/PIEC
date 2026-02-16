"""
Statistical tests module for analyzing experimental results.
"""

import numpy as np
import pandas as pd
from scipy import stats
from typing import Dict, Tuple, List


class StatisticalTests:
    """Perform statistical tests on experimental data."""
    
    def __init__(self):
        """Initialize statistical tests."""
        pass
    
    def energy_anova(self, energy_dict: Dict[str, np.ndarray], alpha: float = 0.05) -> Dict:
        """
        Perform one-way ANOVA on energy consumption across methods.
        
        H0: μ_PIEC = μ_A* = μ_RRT* = μ_DWA = μ_NSGA-II-Std
        
        Args:
            energy_dict: Dictionary mapping method names to energy arrays
            alpha: Significance level
            
        Returns:
            Dictionary with ANOVA results
        """
        # Extract energy arrays
        groups = [energy_dict[method] for method in energy_dict.keys()]
        
        # Perform ANOVA
        F_stat, p_value = stats.f_oneway(*groups)
        
        # Calculate degrees of freedom
        k = len(groups)  # number of groups
        n = sum(len(g) for g in groups)  # total sample size
        df_between = k - 1
        df_within = n - k
        df_total = n - 1
        
        # Calculate sum of squares
        grand_mean = np.mean(np.concatenate(groups))
        
        # Between-group sum of squares
        ss_between = sum(len(g) * (np.mean(g) - grand_mean)**2 for g in groups)
        
        # Within-group sum of squares
        ss_within = sum(np.sum((g - np.mean(g))**2) for g in groups)
        
        # Total sum of squares
        ss_total = ss_between + ss_within
        
        # Mean squares
        ms_between = ss_between / df_between
        ms_within = ss_within / df_within
        
        return {
            'F_statistic': F_stat,
            'p_value': p_value,
            'significant': p_value < alpha,
            'alpha': alpha,
            'ss_between': ss_between,
            'ss_within': ss_within,
            'ss_total': ss_total,
            'df_between': df_between,
            'df_within': df_within,
            'df_total': df_total,
            'ms_between': ms_between,
            'ms_within': ms_within
        }
    
    def tukey_hsd(self, energy_dict: Dict[str, np.ndarray], alpha: float = 0.05) -> pd.DataFrame:
        """
        Perform Tukey HSD post-hoc test for pairwise comparisons.
        
        Args:
            energy_dict: Dictionary mapping method names to energy arrays
            alpha: Significance level
            
        Returns:
            DataFrame with pairwise comparison results
        """
        try:
            from statsmodels.stats.multicomp import pairwise_tukeyhsd
            
            # Prepare data
            all_energies = []
            method_labels = []
            
            for method, energies in energy_dict.items():
                all_energies.extend(energies)
                method_labels.extend([method] * len(energies))
            
            # Perform Tukey HSD
            tukey_result = pairwise_tukeyhsd(all_energies, method_labels, alpha=alpha)
            
            # Convert to DataFrame
            results_df = pd.DataFrame({
                'group1': tukey_result.groupsunique[tukey_result._results_table.data[1:, 0].astype(int)],
                'group2': tukey_result.groupsunique[tukey_result._results_table.data[1:, 1].astype(int)],
                'meandiff': tukey_result._results_table.data[1:, 2],
                'lower': tukey_result._results_table.data[1:, 3],
                'upper': tukey_result._results_table.data[1:, 4],
                'reject': tukey_result._results_table.data[1:, 5]
            })
            
            return results_df
            
        except ImportError:
            print("Warning: statsmodels not available, performing manual pairwise t-tests")
            return self._manual_pairwise_tests(energy_dict, alpha)
    
    def _manual_pairwise_tests(self, energy_dict: Dict[str, np.ndarray], 
                               alpha: float = 0.05) -> pd.DataFrame:
        """Perform manual pairwise t-tests with Bonferroni correction."""
        methods = list(energy_dict.keys())
        results = []
        
        # Bonferroni correction
        n_comparisons = len(methods) * (len(methods) - 1) // 2
        corrected_alpha = alpha / n_comparisons
        
        for i in range(len(methods)):
            for j in range(i + 1, len(methods)):
                method1 = methods[i]
                method2 = methods[j]
                
                data1 = energy_dict[method1]
                data2 = energy_dict[method2]
                
                # Perform t-test
                t_stat, p_value = stats.ttest_ind(data1, data2)
                
                # Calculate confidence interval for mean difference
                mean_diff = np.mean(data1) - np.mean(data2)
                se = np.sqrt(np.var(data1)/len(data1) + np.var(data2)/len(data2))
                ci = stats.t.interval(1-alpha, len(data1)+len(data2)-2, 
                                     loc=mean_diff, scale=se)
                
                results.append({
                    'group1': method1,
                    'group2': method2,
                    'meandiff': mean_diff,
                    'lower': ci[0],
                    'upper': ci[1],
                    'p_value': p_value,
                    'reject': p_value < corrected_alpha
                })
        
        return pd.DataFrame(results)
    
    def cohens_d(self, group1: np.ndarray, group2: np.ndarray) -> float:
        """
        Calculate Cohen's d effect size.
        
        Args:
            group1: First group data
            group2: Second group data
            
        Returns:
            Cohen's d value
        """
        n1, n2 = len(group1), len(group2)
        var1, var2 = np.var(group1, ddof=1), np.var(group2, ddof=1)
        pooled_std = np.sqrt(((n1-1)*var1 + (n2-1)*var2) / (n1+n2-2))
        
        return (np.mean(group1) - np.mean(group2)) / pooled_std
    
    def cohens_d_analysis(self, energy_dict: Dict[str, np.ndarray], 
                          baseline: str = 'PIEC') -> pd.DataFrame:
        """
        Calculate Cohen's d for all methods compared to baseline.
        
        Args:
            energy_dict: Dictionary mapping method names to energy arrays
            baseline: Baseline method name
            
        Returns:
            DataFrame with effect size analysis
        """
        results = []
        
        if baseline not in energy_dict:
            raise ValueError(f"Baseline method '{baseline}' not found in data")
        
        baseline_data = energy_dict[baseline]
        
        for method, data in energy_dict.items():
            if method == baseline:
                continue
            
            d = self.cohens_d(baseline_data, data)
            
            # Interpret effect size
            if abs(d) < 0.2:
                interpretation = 'negligible'
            elif abs(d) < 0.5:
                interpretation = 'small'
            elif abs(d) < 0.8:
                interpretation = 'medium'
            else:
                interpretation = 'large'
            
            results.append({
                'comparison': f'{baseline} vs {method}',
                'cohens_d': d,
                'abs_d': abs(d),
                'interpretation': interpretation
            })
        
        return pd.DataFrame(results)
    
    def shapiro_wilk_test(self, data: np.ndarray, alpha: float = 0.05) -> Dict:
        """
        Perform Shapiro-Wilk test for normality.
        
        Args:
            data: Data array
            alpha: Significance level
            
        Returns:
            Dictionary with test results
        """
        stat, p_value = stats.shapiro(data)
        
        return {
            'statistic': stat,
            'p_value': p_value,
            'is_normal': p_value > alpha,
            'alpha': alpha
        }
    
    def paired_ttest(self, group1: np.ndarray, group2: np.ndarray, 
                     alpha: float = 0.05) -> Dict:
        """
        Perform paired t-test.
        
        Args:
            group1: First group data (must be paired with group2)
            group2: Second group data
            alpha: Significance level
            
        Returns:
            Dictionary with test results
        """
        if len(group1) != len(group2):
            raise ValueError("Groups must have same length for paired t-test")
        
        t_stat, p_value = stats.ttest_rel(group1, group2)
        
        # Calculate confidence interval
        diff = group1 - group2
        mean_diff = np.mean(diff)
        se_diff = stats.sem(diff)
        ci = stats.t.interval(1-alpha, len(diff)-1, loc=mean_diff, scale=se_diff)
        
        return {
            't_statistic': t_stat,
            'p_value': p_value,
            'significant': p_value < alpha,
            'mean_difference': mean_diff,
            'ci_lower': ci[0],
            'ci_upper': ci[1],
            'alpha': alpha
        }
    
    def bootstrap_ci(self, data: np.ndarray, statistic=np.mean, 
                     n_resamples: int = 10000, confidence_level: float = 0.95) -> Dict:
        """
        Calculate bootstrap confidence interval.
        
        Args:
            data: Data array
            statistic: Function to calculate statistic (default: mean)
            n_resamples: Number of bootstrap resamples
            confidence_level: Confidence level
            
        Returns:
            Dictionary with bootstrap results
        """
        from scipy.stats import bootstrap
        
        rng = np.random.default_rng()
        res = bootstrap((data,), statistic, n_resamples=n_resamples,
                       confidence_level=confidence_level, random_state=rng)
        
        return {
            'statistic': statistic(data),
            'ci_lower': res.confidence_interval.low,
            'ci_upper': res.confidence_interval.high,
            'confidence_level': confidence_level,
            'n_resamples': n_resamples
        }
