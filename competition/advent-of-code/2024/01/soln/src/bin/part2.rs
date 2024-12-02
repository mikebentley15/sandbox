use counter::Counter;
use soln::common::*;

fn main() {
    let fname = get_input_file();
    match read_lines(fname) {
        Err(why) => panic!("Could not read lines: {}", why),
        Ok(lines) => {
            let (a, b) = iterate_lines(lines).unzip::<i32, i32, Counter<i32>, Counter<i32>>();
            let total_score = a
                .keys()
                .fold(0, |total, key| total + (*key as usize) * a[&key] * b[&key]);
            println!("Total score: {}", total_score);
        }
    }
}
