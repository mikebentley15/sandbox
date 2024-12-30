use std::collections::HashSet;
use std::env;
use std::ffi::OsString;
use std::fs::File;
use std::io::{self, BufRead};
use std::iter::Iterator;
use std::path::Path;

use crate::multimap::*;

pub type PageNo = u64;
pub type ConstraintMap = Multimap<PageNo, PageNo>;
pub type Proposal = Vec<PageNo>;

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

pub fn create_map_and_proposals<I: Iterator<Item = String>>(
    lines: I,
) -> Result<(ConstraintMap, Vec<Proposal>), String> {
    let mut map = ConstraintMap::new();
    let mut proposals = Vec::new();
    for line in lines {
        if line.contains('|') {
            // is constraint
            let maybe_splitvals: Result<Vec<PageNo>, _> =
                line.split('|').map(|s| s.parse::<PageNo>()).collect();
            match maybe_splitvals {
                Ok(splitvals) => {
                    if splitvals.len() != 2 {
                        return Err(String::from("Incorrect number of map terms"));
                    }
                    map.insert(splitvals[0], splitvals[1]);
                }
                Err(e) => return Err(format!("Parse error: {}", e)),
            };
        } else if line.contains(',') {
            // is proposal
            let maybe_splitvals: Result<Vec<PageNo>, _> =
                line.split(',').map(|s| s.parse::<PageNo>()).collect();
            match maybe_splitvals {
                Ok(splitvals) => proposals.push(splitvals),
                Err(e) => return Err(format!("Parse error: {}", e)),
            };
        } else {
            // do nothing
        }
    }
    return Ok((map, proposals));
}

pub fn is_proposal_valid(proposal: &Proposal, constraints: &ConstraintMap) -> bool {
    let mut remaining_pages: HashSet<PageNo> = HashSet::from_iter(proposal.iter().cloned());
    for page in proposal.iter().rev() {
        remaining_pages.remove(&page);
        let is_valid = constraints
            .data
            .get(page)
            .map(|cset| cset.is_disjoint(&remaining_pages))
            .unwrap_or(true);
        if !is_valid {
            return false;
        }
    }
    return true;
}

pub fn middle_value<T>(seq: &Vec<T>) -> Option<&T> {
    seq.get(seq.len() / 2)
}
