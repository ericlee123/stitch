package com.stitch.core

import boofcv.abst.feature.detdesc.DetectDescribePoint
import boofcv.abst.feature.detect.interest.ConfigFastHessian
import boofcv.alg.descriptor.UtilFeature
import boofcv.factory.feature.associate.FactoryAssociation
import boofcv.factory.feature.detdesc.FactoryDetectDescribe
import boofcv.gui.feature.AssociationPanel
import boofcv.gui.image.ShowImages
import boofcv.io.image.ConvertBufferedImage
import boofcv.struct.feature.BrightFeature
import boofcv.struct.image.GrayF32
import georegression.struct.point.Point2D_F64
import java.io.File
import java.util
import javax.imageio.ImageIO
import org.ddogleg.struct.FastQueue

/**
 * Created by ashwin on 3/1/17.
 */
object BoofExample {

  def main(args: Array[String]): Unit = {
    // Construct our stitcher using this interface which was crafted by a second grader on meth.
    val detector: DetectDescribePoint[GrayF32, BrightFeature] = FactoryDetectDescribe.surfStable(
      new ConfigFastHessian(1, 2, 300, 1, 9, 4, 4),
      null,
      null,
      classOf[GrayF32]
    )

    val associator = FactoryAssociation.greedy(
      FactoryAssociation.defaultScore(detector.getDescriptionType),
      Double.MaxValue,
      true
    )

    def describe(input: GrayF32): (util.List[Point2D_F64], FastQueue[BrightFeature]) = {
      val points = new util.ArrayList[Point2D_F64]()
      val descriptors = UtilFeature.createQueue(detector, 100)

      detector.detect(input)
      (0 until detector.getNumberOfFeatures) foreach { i =>
        points.add(detector.getLocation(i).copy())
        descriptors.grow().setTo(detector.getDescription(i))
      }

      (points, descriptors)
    }

    // Use it to stitch some shit.
    val imA = ImageIO.read(new File("stitch-assets/kitchen-0.jpg"))
    val imB = ImageIO.read(new File("stitch-assets/kitchen-1.jpg"))

    val inA = ConvertBufferedImage.convertFromSingle(imA, null, classOf[GrayF32])
    val inB = ConvertBufferedImage.convertFromSingle(imB, null, classOf[GrayF32])

    // Find location, and descriptors of interest points in the provided input.
    val (ptA, descA) = describe(inA)
    val (ptB, descB) = describe(inB)

    associator.setSource(descA)
    associator.setDestination(descB)
    associator.associate()

    val panel = new AssociationPanel(20)
    panel.setAssociation(ptA, ptB, associator.getMatches())
    panel.setImages(imA, imB)
    ShowImages.showWindow(panel,"Associated Features", true)
  }

}
