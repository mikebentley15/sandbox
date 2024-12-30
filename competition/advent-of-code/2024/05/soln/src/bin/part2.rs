use soln::common::*;
use std::cmp::Ordering;

fn main() -> Result<(), String> {
    let fname = get_input_file();
    let lines = read_lines(&fname).unwrap().map(|x| x.unwrap());
    let (constraints, proposals) = create_map_and_proposals(lines)?;
    let proposal_count = proposals.len();

    let invalid_proposals: Vec<Proposal> = proposals
        .into_iter()
        .filter(|p| {
            let is_valid = is_proposal_valid(p, &constraints);
            let validstr = if is_valid { " valid " } else { "INVALID" };
            println!("Proposal is {}: {:?}", validstr, p);
            !is_valid
        })
        .collect();

    let invalid_count = invalid_proposals.len();
    let valid_count = proposal_count - invalid_count;
    println!("");
    println!("Proposal Summary:");
    println!("  Count:   {}", proposal_count);
    println!("  Valid:   {}", valid_count);
    println!("  Invalid: {}", invalid_count);
    println!("");

    let total: u64 = invalid_proposals
        .into_iter()
        .map(|mut p| {
            p.sort_by(|a, b| -> Ordering {
                if constraints.contains_key_and_value(a, b) {
                    Ordering::Less
                } else if constraints.contains_key_and_value(b, a) {
                    Ordering::Greater
                } else {
                    Ordering::Equal
                }
            });
            p
        })
        .map(|p| middle_value(&p).cloned().unwrap())
        .sum();
    println!("Total:             {total}");

    Ok(())
}
