pub struct WordGrid {
    data: Vec<u8>,
    width: usize,
    height: usize,
}

impl WordGrid {
    fn new<I: Iterator<Item = String>>(lines: &mut I) -> Result<WordGrid, &str> {
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
            data: data.into_bytes(),
            width,
            height,
        });
    }

    fn at(self, row: usize, col: usize) -> Option<char> {
        return self.row(row)?[col];
    }

    fn row<'a>(&'a self, idx: usize) -> Option<&'a str> {
        if idx >= self.height {
            return None;
        }
        let begin = self.width * idx;
        let end = self.width * (idx + 1);
        return Some(&self.data[begin..end]);
    }

    fn col<'a>(&'a self, idx: usize) -> Option<WordGridCol<'a>> {
        if idx >= self.width {
            return None;
        }
        return Some(WordGridCol {
            grid: &self,
            col: idx,
        });
    }
}

pub struct WordGridCol<'a> {
    grid: &'a WordGrid,
    col: usize,
}

pub struct WordGridColIterator<'a> {
    grid: &'a WordGrid,
    col: usize,
}

pub struct WordGridDiag<'a> {
    grid: &'a WordGrid,
    diag: usize,
}

pub struct WordGridUpDiagIterator<'a> {
    grid: &'a WordGrid,
    diag: usize,
}

pub struct WordGridReverseUpDiagIterator<'a> {
    grid: &'a WordGrid,
    diag: usize,
}

pub struct WordGridDownDiagIterator<'a> {
    grid: &'a WordGrid,
    diag: usize,
}

pub struct WordGridReverseDownDiagIterator<'a> {
    grid: &'a WordGrid,
    diag: usize,
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
        let mut line_iter = lines.iter();
        match WordGrid::new(&mut line_iter) {
            Err(reason) => panic!("{}", reason),
            Ok(grid) => return grid,
        }
    }

    #[test]
    fn test_WordGridAt() {}
}
