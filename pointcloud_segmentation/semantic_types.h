#ifndef MOBILITY_SEMANTIC_TYPES_H_
#define MOBILITY_SEMANTIC_TYPES_H_

namespace mobility::semantic {

// Contains the string that shall be used to represent an object of
// unknown/unsure class in all modules that deal with detection and object
// classes.
inline constexpr char kUnknownClass[] = "not sure";

// Contains a string that shall be used for on robot (self) body points.
inline constexpr char kBodyEgo[] = "ego";

// Contains a string that shall be used for floor cells/points.
inline constexpr char kFloor[] = "floor";

// Contains a string that shall be used for wall cells/points.
inline constexpr char kWall[] = "wall";

// Contains a string that shall be used for table cells/points.
inline constexpr char kTable[] = "table";

}  // namespace mobility::semantic

#endif  // MOBILITY_OBJECT_PROPERTIES_VISUAL_VOCABULARIES_SEMANTIC_TYPES_H_
