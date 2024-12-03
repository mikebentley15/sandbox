use soln::common::*;

fn main() {
    let fname = get_input_file();
    let (mut a, mut b) = create_vectors_from_input(&fname);
    a.sort();
    b.sort();
    let total_distance = a
        .iter()
        .zip(b.iter())
        .fold(0, |total, pair| total + (pair.0 - pair.1).abs());
    println!("Total distance: {}", total_distance);
}
