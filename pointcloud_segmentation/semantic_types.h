/*
 * Copyright 2023 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
