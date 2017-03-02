package com.stitch.core

import boofcv.struct.feature.TupleDesc
import georegression.struct.point.Point2D_F64

/**
 * An interest point.
 *
 * @param location
 * @param description
 * @tparam Desc
 */
case class InterestPoint[Desc <: TupleDesc[Desc]](
  location: Point2D_F64,
  description: Desc
)
