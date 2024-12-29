use soln::common::*;
use soln::wordgrid::*;

fn main() {
    let fname = get_input_file();
    let lines = read_lines(&fname).unwrap().map(|x| x.unwrap());
    let grid = WordGrid::new(lines).unwrap();
    let total = grid.count_xmases();
    println!("  total:             {total}");
}
