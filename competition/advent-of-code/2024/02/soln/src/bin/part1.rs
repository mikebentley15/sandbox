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
    // for optional_report_string in report_strings {
    //     let report_string = optional_report_string.unwrap();
    //     let report = split_int(&report_string);
    //     for (a, b) in report.tuple_windows() {
    //         println!("Pair: {} {}", a, b);
    //     }
    // }
    // println!("Total distance: {}", total_distance);
}

fn is_report_string_safe(report_string: &String) -> bool {
    return is_report_safe(split_int(report_string));
}

fn is_report_safe<I: Iterator<Item = i32>>(report: I) -> bool {
    let begin_pairstat = PairStats {
        is_good_distance: true,
        is_decreasing: true,
        is_increasing: true,
    };
    let reportstat = report
        .tuple_windows::<(_, _)>()
        .map(report_pair_stats)
        .fold(begin_pairstat, |folded, pairstat| PairStats {
            is_good_distance: folded.is_good_distance && pairstat.is_good_distance,
            is_decreasing: folded.is_decreasing && pairstat.is_decreasing,
            is_increasing: folded.is_increasing && pairstat.is_increasing,
        });
    return reportstat.is_good_distance && (reportstat.is_increasing || reportstat.is_decreasing);
}
