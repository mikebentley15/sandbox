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

/// Parse a string in the form mul(A,B) where A and B are 1-3 digit positive numbers.
pub fn parse_mul<S: AsRef<str> + ?Sized>(a_str: &S, b_str: &S) -> usize {
    let a = a_str.as_ref().parse::<usize>().unwrap();
    let b = b_str.as_ref().parse::<usize>().unwrap();
    let mul = a * b;
    println!("{} * {} = {}", a, b, mul);
    return mul;
}
