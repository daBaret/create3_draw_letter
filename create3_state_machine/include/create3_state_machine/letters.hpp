#include <map>
#include <vector>

std::map<char, std::vector<double>> letters = {
  { '!', { -0.7, 0, -0.2, 0 } },
  { 'S', { -567.0, 630.0,  -630.0, 567.0,  -630.0, 567.0,  -662.0, 472.0,  -662.0, 472.0,  -662.0, 346.0,  -662.0,
           346.0,  -630.0, 252.0,  -630.0, 252.0,  -567.0, 189.0,  -567.0, 189.0,  -504.0, 189.0,  -504.0, 189.0,
           -441.0, 220.0,  -441.0, 220.0,  -410.0, 252.0,  -410.0, 252.0,  -378.0, 315.0,  -378.0, 315.0,  -315.0,
           504.0,  -315.0, 504.0,  -284.0, 567.0,  -284.0, 567.0,  -252.0, 598.0,  -252.0, 598.0,  -189.0, 630.0,
           -189.0, 630.0,  -94.5,  630.0,  -94.5,  630.0,  -31.5,  567.0,  -31.5,  567.0,  -0.0,   472.0,  -0.0,
           472.0,  -0.0,   346.0,  -0.0,   346.0,  -31.5,  252.0,  -31.5,  252.0,  -94.5,  189.0 } },
  { 'E',
    { -0.0, 220.0, -662.0, 220.0, -662.0, 630.0, -662.0, 220.0, -346.0, 220.0, -346.0, 472.0, -346.0, 220.0, -0.0,
      220.0, -0.0, 630.0 } },
  { 'N', {-0.0, 220.0, -662.0, 220.0, -0.0, 662.0, -662.0, 662.0, -0.0, 662.0 } },
  { 'V', { -662.0, 126.0, -0.0, 378.0, -662.0, 630.0} }
};

double letter_size = 650;