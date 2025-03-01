#include "lineSegment.h"

#include "../physics_object.h"
#include <math.h>
#include "../../render/defs.h"

void lineSegment_support_function(void* data, Vector3* direction, Vector3* output) {
    line_segment* segment = (line_segment*)data;
    float dotStart = vector3Dot(&segment->segment_start, direction);
    float dotEnd = vector3Dot(&segment->segment_end, direction);
    *output = (dotStart > dotEnd) ? segment->segment_start : segment->segment_end;
}
