/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_MOVEING_AVERAGE_FILTER_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_MOVEING_AVERAGE_FILTER_H_

#include <cstdint>

// 可以理解为滑动窗口滤波器

class MovingAverage
{
private:
  // 指向滑动窗口
  int *values;
  // 窗口大小
  int size;
  // 当前索引值
  int currentIndex;
  // 总值
  int sum;
  // 实际有效个数
  int count;

public:
  // 构造函数
  MovingAverage(int size) : size(size), currentIndex(0), sum(0), count(0)
  {
    values = new int[size]();
  }

  // 析构函数
  ~MovingAverage()
  {
    delete[] values;
  }

  // 添加一个数据
  void addValue(int value)
  {
    if (count < size)
    {
      values[currentIndex] = value;
      sum += value;
      count++;
    }
    else
    {
      sum -= values[currentIndex];
      sum += value;
      values[currentIndex] = value;
    }

    currentIndex = (currentIndex + 1) % size;
  }

  // 获取平均值
  int getMovingAverage() const
  {
    return sum / count;
  }
};

#endif // MODULES_WHIMSYCWD_MARLIN_SRC_CORE_MOVEING_AVERAGE_FILTER_H_
