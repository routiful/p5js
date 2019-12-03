function setup()
{
  createCanvas(400, 400);
  axis.show();
}

function draw()
{

}

class Robot
{
  constructor(radius, xin, yin)
  {

  }

  update()
  {

  }

  move()
  {

  }
}

function axis()
{
  this.show() = function()
  {
    background(255);

    for (let i = 0; i < width; i = i + 10)
    {
      stroke(0);
      strokeWeight(1);
      line(i, 0, i, 10);
    }

    for (let j = 0; j < height; j = j + 10)
    {
      stroke(0);
      line(0, j, 10, j);
    }

    strokeWeight(10);
    stroke(255, 0, 0);
    line(0, 0, 0, height);
    stroke(0, 255, 0);
    line(0, 0, width, 0);
  }
}
