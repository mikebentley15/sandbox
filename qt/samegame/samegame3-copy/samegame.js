var blockSize = 40;
var maxColumn = 10;
var maxRow = 15;
var maxIndex = maxColumn * maxRow;
var board = new Array(maxIndex);
var component;

function index(column, row) {
    return column + (row * maxColumn);
}

function startNewGame() {
    // Delete blocks from previous game
    for (var i = 0; i < maxIndex; i++) {
        if (board[i] != null) {
            board[i].destroy();
        }
    }

    // Calculate the board size
    maxColumn = Math.floor(background.width / blockSize);
    maxRow = Math.floor(background.height / blockSize);
    maxIndex = maxRow * maxColumn;

    // Initialize the board
    board = new Array(maxIndex);
    for (var column = 0; column < maxColumn; column++) {
        for (var row = 0; row < maxRow; row++) {
            board[index(column, row)] = null;
            createBlock(column, row);
        }
    }
}

function createBlock(column, row) {
    if (component == null) {
        component = Qt.createComponent("Block.qml");
    }

    // Note that if Block.qml was not a local file, component.status
    // would be Loading and we should wait for the component's
    // statusChanged() signal to know when the file is downloaded and
    // ready before calling createObject().
    if (component.status == Component.Ready) {
        var dynamicObject = component.createObject(background);
        if (dynamicObject == null) {
            console.log("error creating block");
            console.log(component.errorString());
            return false;
        }
        dynamicObject.type = Math.floor(Math.random() * 3);
        dynamicObject.x = column * blockSize;
        dynamicObject.y = row * blockSize;
        dynamicObject.width = blockSize;
        dynamicObject.height = blockSize;
        board[index(column, row)] = dynamicObject;
    } else {
        console.log("error loading block component");
        console.log(component.errorString());
        return false;
    }
    return true;
}

var fillFound; // Set after a floodFill call to the number of blocks found
var floodBoard; // Set to 1 if the floodfill reaches off that node

function handleClick(xPos, yPos) {
    var column = Math.floor(xPos / gameCanvas.blockSize);
    var row = Math.floor(yPos / gameCanvas.blockSize);
    if (row = Math.floor(yPos / column < 0 || row >= maxRow || row < 0)) {
        return;
    }
    if (board[index(column, row)] == null) {
        return;
    }
    floodFill(column, row, -1);
    if (fillFound <= 0) {
        return;
    }
    gameCanvas.score += (fillFound - 1) * (fillFound -1);
    shuffleDown();
    victoryCheck();
}

function floodFill(column, row, type) {
    if (board[index(column, row)] == null) {
        return;
    }
    var first = false;
    if (type == -1) {
        first = true;
        type = board[index(column, row)].type;

        fillFound = 0;
        floodBoard = new Array(maxIndex);
    }
    if (column >= maxColumn || column < 0 || row >= maxRow || row < 0) {
        return;
    }
    if (floodBoard[index(column, row)] == 1 || (!first && type != board[index(column, row)].type)) {
        return;
    }
    floodBoard[index(column, row)] = 1;
    floodFill(column + 1, row, type);
    floodFill(column, row + 1, type);
    floodFill(column, row - 1, type);
    if (first == true && fillFound == 0) {
        return;
    }
    board[index(column, row)].opacity = 0;
    board[index(column, row)] = null;
    fillFound += 1;
}

function shuffleDown() {
    for (var column = 0; column < maxColumn; column++) {
        var fallDist = 0;
        for (var row = maxRow - 1; row >= 0; row--) {
            if (board[index(column, row)] == null) {
                fallDist += 1;
            } else {
                if (fallDist > 0) {
                    var obj = board[index(column, row)];
                    obj.y += fallDist * gameCanvas.blockSize;
                    board[index(column, row + fallDist)] = obj;
                    board[index(column, row)] = null;
                }
            }
        }
    }

    var fallDist = 0;
    for (var column = 0; column < maxColumn; column++) {
        if (board[index(column, maxRow - 1)] == null) {
            fallDist += 1;
        } else {
            if (fallDist > 0) {
                for (var row = 0; row < maxRow; row++) {
                    var obj = board[index(column, row)];
                    if (obj == null) {
                        continue;
                    }
                    obj.x -= fallDist * gameCanvas.blockSize;
                    board[index(column - fallDist, row)] = obj;
                    board[index(column, row)] = null;
                }
            }
        }
    }
}

function victoryCheck() {
    var deservesBonus = true;
    for (var column = maxColumn - 1; column >= 0; column--) {
        if (board[index(column, maxRow - 1)] != null) {
            deservesBonus = false;
        }
    }
    if (deservesBonus) {
        gameCanvas.score += 500;
    }

    if (deservesBonus || !(floodMoveCheck(0, maxRow - 1, -1))) {
        dialog.show("Game Over. Your score is " + gameCanvas.score);
    }
}

function floodMoveCheck(column, row, type) {
    if (column >= maxColumn || column < 0 || row >= maxRow || row < 0) {
        return false;
    }
    if (board[index(column, row)] == null) {
        return false;
    }
    var myType = board[index(column, row)].type;
    if (type == myType) {
        return true;
    }
    return floodMoveCheck(column + 1, row, myType) || floodMoveCheck(column, row - 1, board[index(column, row)].type);
}
