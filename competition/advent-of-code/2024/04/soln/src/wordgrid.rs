use std::cmp::min;
use std::iter::StepBy;
use std::slice::Iter;

pub fn chain_all<T, Inner: Iterator<Item = T>, Outer: Iterator<Item = Inner>>(
    mut iter_of_iters: Outer,
) -> impl Iterator<Item = T> {
    let first = iter_of_iters.next();
    ChainAll {
        iter_of_iters,
        current_iter: first,
    }
}

pub struct ChainAll<T, Inner: Iterator<Item = T>, Outer: Iterator<Item = Inner>> {
    iter_of_iters: Outer,
    current_iter: Option<Inner>,
}

impl<T, Inner: Iterator<Item = T>, Outer: Iterator<Item = Inner>> Iterator
    for ChainAll<T, Inner, Outer>
{
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_iter.is_none() {
            return None;
        }
        let mut current_iter = self.current_iter.take().unwrap();
        let nextval = current_iter.next();
        if nextval.is_some() {
            self.current_iter = Some(current_iter);
            return nextval;
        }
        while let Some(mut nextiter) = self.iter_of_iters.next() {
            let nextval = nextiter.next();
            if nextval.is_some() {
                self.current_iter = Some(nextiter);
                return nextval;
            }
        }
        // no more iterators
        self.current_iter = None;
        return None;
    }
}

pub fn xmas_count_fast<I: Iterator<Item = u8>>(haystack: I) -> u32 {
    // Implement the search as a manually constructed finite state machine
    enum State {
        None,
        X,
        XM,
        XMA,
        XMAS,
    }
    let mut state = State::None;
    let count = haystack
        .map(|ch| {
            state = match state {
                _ if ch == 'X' as u8 => State::X,
                State::X if ch == 'M' as u8 => State::XM,
                State::XM if ch == 'A' as u8 => State::XMA,
                State::XMA if ch == 'S' as u8 => State::XMAS,
                _ => State::None,
            };
            match state {
                State::XMAS => {
                    state = State::None;
                    1u32
                }
                _ => 0u32,
            }
        })
        .sum();
    count
}

pub fn xmas_count_slow<I: Iterator<Item = u8>>(haystack: I) -> u32 {
    let asvec: Vec<u8> = haystack.collect();
    let full_haystack = String::from_utf8(asvec);
    match full_haystack {
        Ok(content) => content.matches("XMAS").count() as u32,
        Err(_) => 0u32,
    }
}

pub struct WordGrid {
    pub grid: Grid<u8>,
}

impl WordGrid {
    pub fn new<I: Iterator<Item = String>>(mut lines: I) -> Result<WordGrid, &'static str> {
        let mut data = String::new();
        let first = lines.next();
        match first {
            Some(line) => data += &line,
            None => return Err("Grid is empty"),
        }
        let width = data.len();
        let mut height = 1;
        for line in lines {
            if line.len() != width {
                return Err("All rows need to be the same width");
            }
            data += &line;
            height += 1;
        }
        data.shrink_to_fit();
        return Ok(WordGrid {
            grid: Grid {
                data: data.into_bytes(),
                row_count: height,
                col_count: width,
            },
        });
    }

    pub fn width(&self) -> usize {
        return self.grid.col_count;
    }

    pub fn height(&self) -> usize {
        return self.grid.row_count;
    }

    pub fn count_xmases(&self) -> u32 {
        fn do_count<'a, Inner: Iterator<Item = &'a u8>, Outer: Iterator<Item = Inner>>(
            iters: Outer,
        ) -> u32 {
            iters.map(|it| xmas_count_fast(it.copied())).sum()
        }
        fn do_rcount<
            'a,
            Inner: DoubleEndedIterator<Item = &'a u8>,
            Outer: Iterator<Item = Inner>,
        >(
            iters: Outer,
        ) -> u32 {
            do_count(iters.map(|it| it.rev()))
        }
        //let do_rcount = |iters| do_count(iters.map(|it| it.rev()));

        let row_count = do_count(self.grid.iter_rows());
        let row_rcount = do_rcount(self.grid.iter_rows());
        let col_count = do_count(self.grid.iter_cols());
        let col_rcount = do_rcount(self.grid.iter_cols());
        let diag_up_count = do_count(self.grid.iter_up_diags());
        let diag_up_rcount = do_rcount(self.grid.iter_up_diags());
        let diag_down_count = do_count(self.grid.iter_down_diags());
        let diag_down_rcount = do_rcount(self.grid.iter_down_diags());
        let total = row_count
            + row_rcount
            + col_count
            + col_rcount
            + diag_up_count
            + diag_up_rcount
            + diag_down_count
            + diag_down_rcount;

        // let total: usize = mul_lines.map(|line| sum_of_muls(line.unwrap())).sum();
        // println!("\nTotals to {}", total);
        println!("Part 1");
        println!("  row count:         {row_count}");
        println!("  row rcount:        {row_rcount}");
        println!("  col count:         {col_count}");
        println!("  col rcount:        {col_rcount}");
        println!("  diag up count:     {diag_up_count}");
        println!("  diag up rcount:    {diag_up_rcount}");
        println!("  diag down count:   {diag_down_count}");
        println!("  diag down rcount:  {diag_down_rcount}");

        total
    }

    pub fn count_cross_mases(&self) -> u32 {
        // 1. Find candidate A's, one away from the border
        // 2. Check each candidate A for the patterns
        //      M.M  M.S  S.S  S.M
        //      .A.  .A.  .A.  .A.
        //      S.S  M.S  M.M  S.M

        // order: (top-left, top-right, bottom-left, bottom-right)
        let get_cross_neighbors = |irow, icol| {
            (
                *self.grid.at(irow - 1, icol - 1).unwrap(),
                *self.grid.at(irow - 1, icol + 1).unwrap(),
                *self.grid.at(irow + 1, icol - 1).unwrap(),
                *self.grid.at(irow + 1, icol + 1).unwrap(),
            )
        };

        const M: u8 = 'M' as u8;
        const A: u8 = 'A' as u8;
        const S: u8 = 'S' as u8;
        let inner_rows = self
            .grid
            .iter_rows()
            .enumerate()
            .skip(1)
            .take(self.grid.row_count - 2);
        let candidate_cells: Vec<(usize, usize, u8)> = chain_all(inner_rows.map(|(irow, row)| {
            row.enumerate()
                .skip(1)
                .take(self.grid.col_count - 2)
                .map(move |(icol, val)| (irow, icol, *val))
        }))
        .filter(|(_irow, _icol, val)| *val == A)
        .collect();
        let is_candidate_a_cross_mas = |irow, icol| {
            let (tl, tr, bl, br) = get_cross_neighbors(irow, icol);
            return ((tl == M && br == S) || (tl == S && br == M))
                && ((tr == M && bl == S) || (tr == S && bl == M));
        };
        println!("  Candidate cells:  {}", candidate_cells.len());
        let cross_mases = candidate_cells
            .iter()
            .filter(|rcv| is_candidate_a_cross_mas(rcv.0, rcv.1));
        cross_mases.count() as u32
    }
}

pub struct Grid<T> {
    data: Vec<T>,
    row_count: usize,
    col_count: usize,
}

impl<T> Grid<T> {
    pub fn at(&self, row: usize, col: usize) -> Option<&T> {
        if col >= self.col_count {
            None
        } else {
            self.data.get(self.idx(row, col))
        }
    }

    pub fn diag_count(&self) -> usize {
        self.row_count + self.col_count - 1
    }

    pub fn iter_row(&self, idx: usize) -> Iter<'_, T> {
        let start = idx * self.col_count;
        let end = (idx + 1) * self.col_count;
        self.data[start..end].into_iter()
    }

    pub fn iter_col(&self, idx: usize) -> StepBy<Iter<'_, T>> {
        self.data[idx..].into_iter().step_by(self.col_count)
    }

    /// Iterate over a down diagonal.  For a 3x4, the order is specified as such:
    ///    2  3  4  5
    ///    1  2  3  4
    ///    0  1  2  3
    /// where the numbers above represent the diagonal's index (e.g., for idx = 2,
    /// all elements are iterated from top-left to bottom-right where you see 2's).
    ///
    /// The number of diagonals can be obtained from diag_count().  If out of range,
    /// an empty iterator is returned.
    pub fn iter_down_diag(&self, idx: usize) -> StepBy<Iter<'_, T>> {
        let (start, end) = if idx < self.row_count {
            let start_row = self.row_count - idx - 1;
            (
                self.idx(start_row, 0),
                self.clamped_idx(start_row + self.col_count - 1, self.col_count - 1),
            )
        } else {
            let start_col = idx + 1 - self.row_count;
            let end_row = min(self.row_count - 1, self.col_count - 1 - start_col);
            (
                self.clamped_idx(0, start_col),
                self.clamped_idx(end_row, start_col + end_row),
            )
        };
        self.data[start..(end + 1)]
            .into_iter()
            .step_by(self.col_count + 1)
    }

    /// Iterate over an up diagonal.  For a 3x4, the order is specified as such:
    ///    0  1  2  3
    ///    1  2  3  4
    ///    2  3  4  5
    /// where the numbers above represent the diagonal's index (e.g., for idx = 2,
    /// all elements are iterated from top-right to bottom-left where you see 2's).
    ///
    /// The number of diagonals can be obtained from diag_count().  If out of range,
    /// an empty iterator is returned.
    pub fn iter_up_diag(&self, idx: usize) -> StepBy<Iter<'_, T>> {
        let stride = self.col_count - 1;
        let (start, end) = if idx < self.col_count {
            (self.idx(0, idx), self.clamped_idx(idx, 0))
        } else {
            let start_row = idx - stride;
            (
                self.clamped_idx(start_row, stride),
                self.clamped_idx(start_row + stride, 0),
            )
        };
        self.data[start..(end + 1)].into_iter().step_by(stride)
    }

    pub fn iter_rows(&self) -> impl ExactSizeIterator<Item = Iter<'_, T>> {
        // The move here only moves the reference ??
        (0..self.row_count).map(move |idx| self.iter_row(idx))
    }

    pub fn iter_cols(&self) -> impl ExactSizeIterator<Item = StepBy<Iter<'_, T>>> {
        // The move here only moves the reference ??
        (0..self.col_count).map(move |idx| self.iter_col(idx))
    }

    pub fn iter_down_diags(&self) -> impl ExactSizeIterator<Item = StepBy<Iter<'_, T>>> {
        (0..self.diag_count()).map(move |idx| self.iter_down_diag(idx))
    }

    pub fn iter_up_diags(&self) -> impl ExactSizeIterator<Item = StepBy<Iter<'_, T>>> {
        (0..self.diag_count()).map(move |idx| self.iter_up_diag(idx))
    }

    fn len(&self) -> usize {
        self.row_count * self.col_count
    }

    fn idx(&self, i_row: usize, i_col: usize) -> usize {
        i_row * self.col_count + i_col
    }

    fn clamped_idx(&self, i_row: usize, i_col: usize) -> usize {
        min(self.idx(i_row, i_col), self.len() - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_wordgrid_1() -> WordGrid {
        let lines = [
            "..X...".to_string(),
            ".SAMX.".to_string(),
            ".A..A.".to_string(),
            "XMAS.S".to_string(),
            ".X....".to_string(),
        ];
        let line_iter = lines.into_iter();
        match WordGrid::new(line_iter) {
            Err(reason) => panic!("{}", reason),
            Ok(grid) => return grid,
        }
    }

    fn create_tall_test_int_grid() -> Grid<u32> {
        Grid {
            data: (1..31).collect(),
            row_count: 10,
            col_count: 3,
        }
    }

    fn create_short_test_int_grid() -> Grid<u32> {
        Grid {
            data: (1..31).collect(),
            row_count: 3,
            col_count: 10,
        }
    }

    fn create_square_test_int_grid() -> Grid<u32> {
        Grid {
            data: (1..26).collect(),
            row_count: 5,
            col_count: 5,
        }
    }

    #[test]
    fn test_grid_iter_rows_on_tall() {
        let tall = create_tall_test_int_grid();
        let rows_iter = tall.iter_rows();
        let expected_rows = [
            [1, 2, 3],
            [4, 5, 6],
            [7, 8, 9],
            [10, 11, 12],
            [13, 14, 15],
            [16, 17, 18],
            [19, 20, 21],
            [22, 23, 24],
            [25, 26, 27],
            [28, 29, 30],
        ];
        assert_eq!(rows_iter.len(), expected_rows.len());
        for (actual_row, expected_row) in std::iter::zip(rows_iter, expected_rows) {
            assert_eq!(actual_row.cloned().collect::<Vec<u32>>(), expected_row);
        }
    }

    #[test]
    fn test_grid_iter_cols_on_tall() {
        let tall = create_tall_test_int_grid();
        let cols_iter = tall.iter_cols();
        let expected_cols = [
            [1, 4, 7, 10, 13, 16, 19, 22, 25, 28],
            [2, 5, 8, 11, 14, 17, 20, 23, 26, 29],
            [3, 6, 9, 12, 15, 18, 21, 24, 27, 30],
        ];
        assert_eq!(cols_iter.len(), expected_cols.len());
        for (actual_col, expected_col) in std::iter::zip(cols_iter, expected_cols) {
            assert_eq!(actual_col.cloned().collect::<Vec<u32>>(), expected_col);
        }
    }

    #[test]
    fn test_grid_iter_up_diags_on_tall() {
        let tall = create_tall_test_int_grid();
        let up_diags_iter = tall.iter_up_diags();
        let expected_up_diags = [
            Vec::from([1]),
            Vec::from([2, 4]),
            Vec::from([3, 5, 7]),
            Vec::from([6, 8, 10]),
            Vec::from([9, 11, 13]),
            Vec::from([12, 14, 16]),
            Vec::from([15, 17, 19]),
            Vec::from([18, 20, 22]),
            Vec::from([21, 23, 25]),
            Vec::from([24, 26, 28]),
            Vec::from([27, 29]),
            Vec::from([30]),
        ];
        assert_eq!(up_diags_iter.len(), expected_up_diags.len());
        for (actual_up_diag, expected_up_diag) in std::iter::zip(up_diags_iter, expected_up_diags) {
            assert_eq!(
                actual_up_diag.cloned().collect::<Vec<u32>>(),
                expected_up_diag
            );
        }
    }

    #[test]
    fn test_grid_iter_down_diags_on_tall() {
        let tall = create_tall_test_int_grid();
        let down_diags_iter = tall.iter_down_diags();
        let expected_down_diags = [
            Vec::from([28]),
            Vec::from([25, 29]),
            Vec::from([22, 26, 30]),
            Vec::from([19, 23, 27]),
            Vec::from([16, 20, 24]),
            Vec::from([13, 17, 21]),
            Vec::from([10, 14, 18]),
            Vec::from([7, 11, 15]),
            Vec::from([4, 8, 12]),
            Vec::from([1, 5, 9]),
            Vec::from([2, 6]),
            Vec::from([3]),
        ];
        assert_eq!(down_diags_iter.len(), expected_down_diags.len());
        for (actual_down_diag, expected_down_diag) in
            std::iter::zip(down_diags_iter, expected_down_diags)
        {
            assert_eq!(
                actual_down_diag.cloned().collect::<Vec<u32>>(),
                expected_down_diag
            );
        }
    }

    #[test]
    fn test_grid_iter_rows_on_short() {
        let tall = create_short_test_int_grid();
        let rows_iter = tall.iter_rows();
        let expected_rows = [
            [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15, 16, 17, 18, 19, 20],
            [21, 22, 23, 24, 25, 26, 27, 28, 29, 30],
        ];
        assert_eq!(rows_iter.len(), expected_rows.len());
        for (actual_row, expected_row) in std::iter::zip(rows_iter, expected_rows) {
            assert_eq!(actual_row.cloned().collect::<Vec<u32>>(), expected_row);
        }
    }

    #[test]
    fn test_grid_iter_cols_on_short() {
        let tall = create_short_test_int_grid();
        let cols_iter = tall.iter_cols();
        let expected_cols = [
            [1, 11, 21],
            [2, 12, 22],
            [3, 13, 23],
            [4, 14, 24],
            [5, 15, 25],
            [6, 16, 26],
            [7, 17, 27],
            [8, 18, 28],
            [9, 19, 29],
            [10, 20, 30],
        ];
        assert_eq!(cols_iter.len(), expected_cols.len());
        for (actual_col, expected_col) in std::iter::zip(cols_iter, expected_cols) {
            assert_eq!(actual_col.cloned().collect::<Vec<u32>>(), expected_col);
        }
    }

    #[test]
    fn test_grid_iter_up_diags_on_short() {
        let tall = create_short_test_int_grid();
        let up_diags_iter = tall.iter_up_diags();
        let expected_up_diags = [
            Vec::from([1]),
            Vec::from([2, 11]),
            Vec::from([3, 12, 21]),
            Vec::from([4, 13, 22]),
            Vec::from([5, 14, 23]),
            Vec::from([6, 15, 24]),
            Vec::from([7, 16, 25]),
            Vec::from([8, 17, 26]),
            Vec::from([9, 18, 27]),
            Vec::from([10, 19, 28]),
            Vec::from([20, 29]),
            Vec::from([30]),
        ];
        assert_eq!(up_diags_iter.len(), expected_up_diags.len());
        for (actual_up_diag, expected_up_diag) in std::iter::zip(up_diags_iter, expected_up_diags) {
            assert_eq!(
                actual_up_diag.cloned().collect::<Vec<u32>>(),
                expected_up_diag
            );
        }
    }

    #[test]
    fn test_grid_iter_down_diags_on_short() {
        let tall = create_short_test_int_grid();
        let down_diags_iter = tall.iter_down_diags();
        let expected_down_diags = [
            Vec::from([21]),
            Vec::from([11, 22]),
            Vec::from([1, 12, 23]),
            Vec::from([2, 13, 24]),
            Vec::from([3, 14, 25]),
            Vec::from([4, 15, 26]),
            Vec::from([5, 16, 27]),
            Vec::from([6, 17, 28]),
            Vec::from([7, 18, 29]),
            Vec::from([8, 19, 30]),
            Vec::from([9, 20]),
            Vec::from([10]),
        ];
        assert_eq!(down_diags_iter.len(), expected_down_diags.len());
        for (actual_down_diag, expected_down_diag) in
            std::iter::zip(down_diags_iter, expected_down_diags)
        {
            assert_eq!(
                actual_down_diag.cloned().collect::<Vec<u32>>(),
                expected_down_diag
            );
        }
    }

    #[test]
    fn test_grid_iter_rows_on_square() {
        let tall = create_square_test_int_grid();
        let rows_iter = tall.iter_rows();
        let expected_rows = [
            [1, 2, 3, 4, 5],
            [6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15],
            [16, 17, 18, 19, 20],
            [21, 22, 23, 24, 25],
        ];
        assert_eq!(rows_iter.len(), expected_rows.len());
        for (actual_row, expected_row) in std::iter::zip(rows_iter, expected_rows) {
            assert_eq!(actual_row.cloned().collect::<Vec<u32>>(), expected_row);
        }
    }

    #[test]
    fn test_grid_iter_cols_on_square() {
        let tall = create_square_test_int_grid();
        let cols_iter = tall.iter_cols();
        let expected_cols = [
            [1, 6, 11, 16, 21],
            [2, 7, 12, 17, 22],
            [3, 8, 13, 18, 23],
            [4, 9, 14, 19, 24],
            [5, 10, 15, 20, 25],
        ];
        assert_eq!(cols_iter.len(), expected_cols.len());
        for (actual_col, expected_col) in std::iter::zip(cols_iter, expected_cols) {
            assert_eq!(actual_col.cloned().collect::<Vec<u32>>(), expected_col);
        }
    }

    #[test]
    fn test_grid_iter_up_diags_on_square() {
        let tall = create_square_test_int_grid();
        let up_diags_iter = tall.iter_up_diags();
        let expected_up_diags = [
            Vec::from([1]),
            Vec::from([2, 6]),
            Vec::from([3, 7, 11]),
            Vec::from([4, 8, 12, 16]),
            Vec::from([5, 9, 13, 17, 21]),
            Vec::from([10, 14, 18, 22]),
            Vec::from([15, 19, 23]),
            Vec::from([20, 24]),
            Vec::from([25]),
        ];
        assert_eq!(up_diags_iter.len(), expected_up_diags.len());
        for (actual_up_diag, expected_up_diag) in std::iter::zip(up_diags_iter, expected_up_diags) {
            assert_eq!(
                actual_up_diag.cloned().collect::<Vec<u32>>(),
                expected_up_diag
            );
        }
    }

    #[test]
    fn test_grid_iter_down_diags_on_square() {
        let tall = create_square_test_int_grid();
        let down_diags_iter = tall.iter_down_diags();
        let expected_down_diags = [
            Vec::from([21]),
            Vec::from([16, 22]),
            Vec::from([11, 17, 23]),
            Vec::from([6, 12, 18, 24]),
            Vec::from([1, 7, 13, 19, 25]),
            Vec::from([2, 8, 14, 20]),
            Vec::from([3, 9, 15]),
            Vec::from([4, 10]),
            Vec::from([5]),
        ];
        assert_eq!(down_diags_iter.len(), expected_down_diags.len());
        for (actual_down_diag, expected_down_diag) in
            std::iter::zip(down_diags_iter, expected_down_diags)
        {
            assert_eq!(
                actual_down_diag.cloned().collect::<Vec<u32>>(),
                expected_down_diag
            );
        }
    }

    #[test]
    fn test_xmas_count() {
        let string_iter = |s| String::from(s).into_bytes().into_iter();
        let my_xmas_count = |s| xmas_count_fast(string_iter(s));
        assert_eq!(1, my_xmas_count("XMAS"));
        assert_eq!(0, my_xmas_count("ABCD"));
        assert_eq!(0, my_xmas_count("SAMX"));
        assert_eq!(2, my_xmas_count("XMAS.XMAS"));
        assert_eq!(1, my_xmas_count("XXMAS"));
        assert_eq!(2, my_xmas_count("XXMAS..XXMMAASSXXXXMASSSS"));
    }
}
