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
#ifndef __LASER_HEAD_RED_H_
#define __LASER_HEAD_RED_H_

#include "laser_head_20W_40W.h"

#define LASER_RED_2W_CL_OFFSET_X            (0)
#define LASER_RED_2W_CL_OFFSET_Y            (0)



class LaserHeadRed : public LaserHead20W40W {
    public:
        LaserHeadRed() : LaserHead20W40W() {};

        void Init();
        void Loop();




    private:
};



#endif  /**< __LASER_HEAD_RED_H_ */

