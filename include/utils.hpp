#pragma once

#include <cmath>

float AngMod2PI(float ang) {
    while (ang < 0.F) ang += 2.0F * static_cast<float>(M_PI);
    while (ang > 2.F * static_cast<float>(M_PI)) ang -= 2.0F * static_cast<float>(M_PI);
    return ang;
}

float AngModPI(float ang) {
    while (ang < -static_cast<float>(M_PI)) ang += 2.0F * static_cast<float>(M_PI);
    while (ang > static_cast<float>(M_PI)) ang -= 2.0F * static_cast<float>(M_PI);
    return ang;
}