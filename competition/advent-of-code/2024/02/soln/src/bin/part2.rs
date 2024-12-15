use itertools::Itertools;
use soln::common::*;

fn main() {
    let fname = get_input_file();
    let report_strings = read_lines(&fname).unwrap();
    let good_report_count = report_strings
        .map(|x| x.unwrap())
        .filter(|x| is_report_string_safe(x))
        .count();
    println!("Good Report Count: {}", good_report_count);
}

fn is_report_string_safe(report_string: &String) -> bool {
    let report: Vec<i32> = split_int(report_string).collect();
    return is_relaxed_report_safe(report);
}

fn is_relaxed_report_safe(report: Vec<i32>) -> bool {
    let (decrease_violation_count, increase_violation_count) =
        get_report_violation_counts(report.iter().copied());
    let n = report.len();
    let report_but_skip_ith = |idx: usize| {
        return report
            .iter()
            .enumerate()
            .filter(move |val| val.0 != idx)
            .map(|val| *val.1);
    };
    if decrease_violation_count == 0 || increase_violation_count == 0 {
        return true;
    } else if decrease_violation_count <= 2
        && (0..n).any(|idx| get_report_violation_counts(report_but_skip_ith(idx)).0 == 0)
    {
        return true;
    } else if increase_violation_count <= 2
        && (0..n).any(|idx| get_report_violation_counts(report_but_skip_ith(idx)).1 == 0)
    {
        return true;
    }
    return false;
}

fn get_report_violation_counts<I: Iterator<Item = i32>>(report: I) -> (i32, i32) {
    return report
        .tuple_windows::<(_, _)>()
        .map(report_pair_stats)
        .fold((0, 0), |folded, stat| {
            let is_equal = !stat.is_increasing && !stat.is_decreasing;
            if !stat.is_good_distance || is_equal {
                return (folded.0 + 1, folded.1 + 1);
            } else if stat.is_increasing {
                return (folded.0 + 1, folded.1);
            } else {
                // if stat.is_decreasing
                return (folded.0, folded.1 + 1);
            }
        });
}
