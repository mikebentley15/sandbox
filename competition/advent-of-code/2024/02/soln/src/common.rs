use std::env;
use std::ffi::OsString;
use std::fs::File;
use std::io::{self, BufRead};
use std::iter::Iterator;
use std::path::Path;

/// Parses arguments and returns the first argument which is the input file path.
pub fn get_input_file() -> OsString {
    let args: Vec<OsString> = env::args_os().collect();
    if args.len() < 2 {
        panic!("Please specify an input file.");
    }
    if args.len() > 2 {
        panic!("Expecting exactly one input file.");
    }
    return args[1].clone();
}

/// The output is wrapped in a Result to allow matching on errors.
/// Returns an Iterator to the Reader of the lines of the file.
pub fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
    P: AsRef<Path>,
{
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}

pub fn split_int(int_with_strings: &String) -> impl Iterator<Item = i32> + use<'_> {
    let parts = int_with_strings.split_whitespace();
    return parts.map(|intstr| intstr.parse::<i32>().unwrap());
}

pub struct PairStats {
    pub is_good_distance: bool,
    pub is_decreasing: bool,
    pub is_increasing: bool,
}

pub fn report_pair_stats(pair: (i32, i32)) -> PairStats {
    let (a, b) = pair;
    return PairStats {
        is_good_distance: (a - b).abs() < 4,
        is_decreasing: a > b,
        is_increasing: a < b,
    };
}
