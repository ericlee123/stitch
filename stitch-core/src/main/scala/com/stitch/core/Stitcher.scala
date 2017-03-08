package com.stitch.core

import boofcv.abst.feature.associate.AssociateDescription
import boofcv.abst.feature.detdesc.DetectDescribePoint
import boofcv.abst.feature.detect.interest.ConfigFastHessian
import boofcv.alg.descriptor.UtilFeature
import boofcv.alg.distort.PixelTransformHomography_F32
import boofcv.alg.distort.impl.DistortSupport
import boofcv.core.image.border.BorderType
import boofcv.factory.feature.associate.FactoryAssociation
import boofcv.factory.feature.detdesc.FactoryDetectDescribe
import boofcv.factory.geo.{ConfigRansac, FactoryMultiViewRobust}
import boofcv.factory.interpolate.FactoryInterpolation
import boofcv.gui.image.ShowImages
import boofcv.io.image.ConvertBufferedImage
import boofcv.struct.feature.{AssociatedIndex, BrightFeature}
import boofcv.struct.geo.AssociatedPair
import boofcv.struct.image.{GrayF32}
import georegression.struct.homography.Homography2D_F64
import georegression.struct.point.Point2D_F64
import java.awt.image.BufferedImage
import org.ddogleg.fitting.modelset.ModelMatcher
import org.ddogleg.struct.FastQueue
import scala.collection.mutable
import collection.JavaConverters._
import java.io.File
import javax.imageio.ImageIO

object Stitcher {

  def main(args: Array[String]): Unit = {
    // Setup the interest point descriptor, associator, and matcher.
    val descriptor = FactoryDetectDescribe.surfStable(
        new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4),
        null,
        null,
        classOf[GrayF32]
    )

    val associator = FactoryAssociation.greedy(
        FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true),
        2,
        true
    )

    val matcher =
      FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))

    // do the stitching
    val width = 1920
    val height = 1080
    var mainFrame = ImageIO.read(new File("stitch-assets/balcony-2.jpg"))

    var scaledWidth = (height.toDouble / mainFrame.getHeight()) * mainFrame
        .getWidth()
    scaledWidth = Math.floor(scaledWidth)

    var resized =
      new BufferedImage(scaledWidth.toInt, 1080, BufferedImage.TYPE_INT_RGB)
    resized
      .createGraphics()
      .drawImage(mainFrame, 0, 0, scaledWidth.toInt, 1080, null)

    var wideFrame =
      new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB)
    val offset = (width - resized.getWidth()).toDouble / 2
    wideFrame
      .createGraphics()
      .drawImage(resized,
                 offset.toInt,
                 0,
                 resized.getWidth(),
                 resized.getHeight(),
                 null)

    var stitching = stitch(descriptor, associator, matcher)(
        wideFrame,
        ImageIO.read(new File("stitch-assets/balcony-3.jpg")),
        1.0
    )

    stitching = stitch(descriptor, associator, matcher)(
        stitching,
        ImageIO.read(new File("stitch-assets/balcony-1.jpg")),
        1.0
    )

    ImageIO.write(stitching, "jpg", new File("looks-good.jpg"))

//    stitching = stitch(descriptor, associator, matcher)(
//        stitching,
//        ImageIO.read(new File("stitch-assets/balcony-3.jpg")),
//        1.0
//    )
//
//    stitching = stitch(descriptor, associator, matcher)(
//        stitching,
//        ImageIO.read(new File("stitch-assets/balcony-0.jpg")),
//        1.0
//    )

//    var i = 0
//    for (i <- 2 to 2) {
//      stitching = stitch(descriptor, associator, matcher)(
//          stitching,
//          ImageIO.read(new File("stitch-assets/balcony-" + i + ".jpg")),
//          1.0
//      )
//    }
//    ImageIO.write(stitching, "jpg", new File("looks-good.jpg"))
  }

  /**
    *
    * @param inputA
    * @param inputB
    * @param descriptor
    * @param associator
    * @param matcher
    * @return
    */
  def homography(
      descriptor: DetectDescribePoint[GrayF32, BrightFeature],
      associator: AssociateDescription[BrightFeature],
      matcher: ModelMatcher[Homography2D_F64, AssociatedPair]
  )(
      inputA: GrayF32,
      inputB: GrayF32
  ): Homography2D_F64 = {
    // Locate a matching between the interest points of the images.
    val (pointsA, descA) = describe(descriptor)(inputA)
    val (pointsB, descB) = describe(descriptor)(inputB)
    val matches = matching(associator)(descA, descB)

    // Construct an association between points.
    val pairs = mutable.Buffer.empty[AssociatedPair]
    (0 until matches.size).map(matches.get).foreach { m =>
      pairs += new AssociatedPair(pointsA(m.src), pointsB(m.dst), false)
    }

    // Attempt to construct the homography.
    if (!matcher.process(pairs.asJava))
      throw new RuntimeException("Unable to determine homography.")
    else
      matcher.getModelParameters.copy()
  }

  /**
    *
    * @param detector
    * @param input
    * @return
    */
  def describe(
      detector: DetectDescribePoint[GrayF32, BrightFeature]
  )(
      input: GrayF32
  ): (Seq[Point2D_F64], FastQueue[BrightFeature]) = {
    val points = mutable.Buffer.empty[Point2D_F64]
    val descriptors = UtilFeature.createQueue(detector, 100)

    detector.detect(input)
    (0 until detector.getNumberOfFeatures) foreach { i =>
      points += detector.getLocation(i).copy()
      descriptors.grow().setTo(detector.getDescription(i))
    }

    (points, descriptors)
  }

  /**
    *
    * @param descriptorsA
    * @param descriptorsB
    * @return
    */
  def matching(
      associator: AssociateDescription[BrightFeature]
  )(
      descriptorsA: FastQueue[BrightFeature],
      descriptorsB: FastQueue[BrightFeature]
  ): FastQueue[AssociatedIndex] = {
    associator.setSource(descriptorsA)
    associator.setDestination(descriptorsB)
    associator.associate()
    associator.getMatches
  }

  /**
    *
    * http://boofcv.org/index.php?title=Example_Image_Stitching
    *
    * @param descriptor
    * @param associator
    * @param matcher
    * @param imageA
    * @param imageB
    * @param scale
    * @return
    */
  def stitch(
      descriptor: DetectDescribePoint[GrayF32, BrightFeature],
      associator: AssociateDescription[BrightFeature],
      matcher: ModelMatcher[Homography2D_F64, AssociatedPair]
  )(
      imageA: BufferedImage,
      imageB: BufferedImage,
      scale: Double
  ): BufferedImage = {
    // Convert the images to the proper image format.
    val inputA =
      ConvertBufferedImage.convertFromSingle(imageA, null, classOf[GrayF32])
    val inputB =
      ConvertBufferedImage.convertFromSingle(imageB, null, classOf[GrayF32])

    // Convert into a colorized format.
    val colorA = ConvertBufferedImage
      .convertFromMulti(imageA, null, true, classOf[GrayF32])
    val colorAcopy = ConvertBufferedImage
      .convertFromMulti(imageA, null, true, classOf[GrayF32])
    val colorB = ConvertBufferedImage
      .convertFromMulti(imageB, null, true, classOf[GrayF32])

    // Calculate the transform from the image to the output image.
//    val a2o = new Homography2D_F64(scale,
//                                   0,
//                                   colorA.width / 4,
//                                   0,
//                                   scale,
//                                   colorA.height / 4,
//                                   0,
//                                   0,
//                                   1)
//    val o2a = a2o.invert(null)
//    val a2b = homography(descriptor, associator, matcher)(inputA, inputB)
//    val o2b = o2a.concat(a2b, null)
//    val b2o = o2b.invert(null)
    val a2b = homography(descriptor, associator, matcher)(inputA, inputB)
    val a2a = homography(descriptor, associator, matcher)(inputA, inputA) // for overlap

    // Setup the rendering toolchain.
    val model = new PixelTransformHomography_F32
    val interpolater =
      FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport
      .createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    // Construct the stitched image by rendering each image using the homographies.
//    val output = colorA.createSameShape()
    model.set(a2b)
    distortion.apply(colorB, colorA)
    model.set(a2a)
    distortion.apply(colorAcopy, colorA)
//    model.set(o2a)
//    distortion.apply(colorA, output)
//    model.set(o2b)
//    distortion.apply(colorB, output)

    // Convert the output image to a BufferedImage.
    val stitched =
      new BufferedImage(colorA.width, colorA.height, imageA.getType)
    ConvertBufferedImage.convertTo(colorA, stitched, true)
  }

}
