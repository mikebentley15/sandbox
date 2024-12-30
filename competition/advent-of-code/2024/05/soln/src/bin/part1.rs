use soln::common::*;

fn main() -> Result<(), String> {
    let fname = get_input_file();
    let lines = read_lines(&fname).unwrap().map(|x| x.unwrap());
    let (constraints, proposals) = create_map_and_proposals(lines)?;
    let total: u64 = proposals
        .into_iter()
        .filter(|p| {
            let is_valid = is_proposal_valid(p, &constraints);
            let validstr = if is_valid { " valid " } else { "INVALID" };
            println!("Proposal is {}: {:?}", validstr, p);
            is_valid
        })
        .map(|p| middle_value(&p).cloned().unwrap())
        .sum();
    println!("  total:             {total}");

    Ok(())
}
