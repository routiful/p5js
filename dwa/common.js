function Axis()
{
  this.show = function(x, y)
  {
    background(255);

    for (let i = 0, k = 0; i < x; i = i + 10, k++)
    {
      stroke(0);
      strokeWeight(1);

      if (k % 5 == 0)
      {
        line(i, 0, i, 15);
      }
      else
      {
        line(i, 0, i, 10);
      }
    }

    for (let j = 0, k = 0; j < y; j = j + 10, k++)
    {
      stroke(0);
      strokeWeight(1);

      if (k % 5 == 0)
      {
        line(0, j, 15, j);
      }
      else
      {
        line(0, j, 10, j);
      }
    }

    stroke(255, 0, 0); // Red
    strokeWeight(10);
    line(0, 0, x, 0);

    stroke(0, 255, 0); // Blue
    strokeWeight(10);
    line(0, 0, 0, y);

    noStroke();
  }
}

function radians(deg)
{
  return deg * Math.PI / 180;
}

function fmod(x,n)
{
  if (x > n)
    return x - n;
  else
    return x;
}

function normalize_angle_positive(angle)
{
  return fmod(fmod(angle, 2.0 * Math.PI) + 2.0 * Math.PI, 2.0 * Math.PI);
}

function normalize_angle(angle)
{
  let a = normalize_angle_positive(angle);
  if (a > Math.PI)
  {
    a -= 2.0 * Math.PI;
  }

  return a;
}

