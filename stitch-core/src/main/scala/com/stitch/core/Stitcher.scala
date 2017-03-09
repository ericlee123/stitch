package com.stitch.core

import boofcv.abst.feature.associate.AssociateDescription
import boofcv.abst.feature.detdesc.DetectDescribePoint
import boofcv.abst.feature.detect.interest.ConfigFastHessian
import boofcv.alg.descriptor.UtilFeature
import boofcv.alg.distort.{PixelTransformHomography_F32, PointTransformHomography_F32}
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
import boofcv.struct.image.{Color3_F32, GrayF32, ImageType, Planar}
import georegression.struct.homography.Homography2D_F64
import georegression.struct.point.Point2D_F64
import java.awt.image.BufferedImage

import org.ddogleg.fitting.modelset.ModelMatcher
import org.ddogleg.struct.FastQueue

import scala.collection.mutable
import collection.JavaConverters._
import java.io.File
import javax.imageio.ImageIO

import scala.collection.mutable.ListBuffer

object Stitcher {

  def main(args: Array[String]): Unit = {

    // process video into sequence of frames
    val frameSequence = loadVideoFrames("stitch-assets/axis-allies.mp4")

    val width = 1920
    val height = 1080

    generateStitchedImages(frameSequence, width, height)
  }

  def generateStitchedImages(
      frames: Seq[BufferedImage],
      width: Int,
      height: Int
  ): Unit = {
    val lookFrames = 300
    val jumpFrames = 50
    for (i <- 400 to 400) {
      ImageIO.write(
          stitchFrame(i, frames, lookFrames, jumpFrames, width, height),
          "png",
          new File("stitched-video-frames/image-" + "%03d".format(i) + ".png"))
    }
  }

  def loadVideoFrames(
      videoPath: String
  ): Seq[BufferedImage] = {
    // generate frames using ffmpeg (because build and other tools are fucked)
//    print("generating video frames...")
//    new ProcessBuilder("mkdir", "video-frames").start()
//    val pb = new ProcessBuilder("ffmpeg",
//                                "-i",
//                                videoPath,
//                                "video-frames/image-%03d.png")
//    val p = pb.start()
//    p.waitFor();
//    println("done")

    // load images into memory
    println("loading frames into memory...")
    var videoFrames = List[BufferedImage]()
    val dir = new File("video-frames")
    val frameImages = dir.listFiles()
    (1 to frameImages.length).map(i =>
          ImageIO.read(
              new File("video-frames/image-" + "%03d".format(i) + ".png")))
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

  def getSizedFrame(
      frame: BufferedImage,
      width: Int,
      height: Int
  ): BufferedImage = {

    // TODO: don't assume that the black bars are on the side (safe assumption though)
    var scaledWidth = Math
      .floor((height.toDouble / frame.getHeight()) * frame.getWidth())
      .toInt

    var resized =
      new BufferedImage(scaledWidth, height, BufferedImage.TYPE_INT_RGB)
    resized.createGraphics().drawImage(frame, 0, 0, scaledWidth, height, null)

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
    wideFrame
  }

  def stitchFrame(
      index: Int,
      frames: Seq[BufferedImage],
      lookFrames: Int,
      jumpFrames: Int,
      width: Int,
      height: Int
  ): BufferedImage = {

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

    var mainFrame = getSizedFrame(frames(index), width, height)
    for (i <- 1 to lookFrames by jumpFrames) {
      if (index - i >= 0) {
        mainFrame = stitch(descriptor, associator, matcher)(
            mainFrame,
            frames(index - i),
            1.0
        )
      }
      if (index + i < frames.length) {
        mainFrame = stitch(descriptor, associator, matcher)(
            mainFrame,
            frames(index + i),
            1.0
        )
      }
    }
    mainFrame

//    var surroundingFrames = ListBuffer[BufferedImage]()
//    for (i <- lookFrames to 1 by -jumpFrames) {
//      if (index + i < frames.length) {
//        surroundingFrames += frames(index + i)
//      }
//      if (index - i >= 0) {
//        surroundingFrames += frames(index - i)
//      }
//    }
//    println(surroundingFrames.length + " surrounding frames")
//
//    val mainFrame = getSizedFrame(frames(index), width, height)
//
//    // Convert the images to the proper image format
//    val mainInput =
//      ConvertBufferedImage.convertFromSingle(mainFrame, null, classOf[GrayF32])
//    val inputs = ListBuffer[GrayF32]()
//    for (i <- 0 to surroundingFrames.length - 1) {
//      inputs += ConvertBufferedImage
//        .convertFromSingle(surroundingFrames(i), null, classOf[GrayF32])
//    }
//
//    // Convert to a colorized format
//    var mainColor = ConvertBufferedImage
//      .convertFromMulti(mainFrame, null, true, classOf[GrayF32])
//    val colors = ListBuffer[Planar[GrayF32]]()
//    for (i <- 0 to surroundingFrames.length - 1) {
//      colors += ConvertBufferedImage
//        .convertFromMulti(surroundingFrames(i), null, true, classOf[GrayF32])
//    }
//
//    val homographies2Main = ListBuffer[Homography2D_F64]()
//    for (i <- 0 to surroundingFrames.length - 1) {
//      homographies2Main += homography(descriptor, associator, matcher)(
//          mainInput,
//          inputs(i))
//    }
//
//    // Setup the rendering toolchain.
//    val model = new PixelTransformHomography_F32
//    val interpolater =
//      FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
//    val distortion = DistortSupport
//      .createDistortPL(classOf[GrayF32], model, interpolater, false)
//    distortion.setRenderAll(false)
//
//    for (i <- 0 to surroundingFrames.length - 1) {
//      model.set(homographies2Main(i))
//      distortion.apply(colors(i), mainColor)
//    }
//
//    val stitched = new BufferedImage(mainFrame.getWidth,
//                                     mainFrame.getHeight(),
//                                     mainFrame.getType())
//    ConvertBufferedImage.convertTo(mainColor, stitched, true)
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
//    val a2a = homography(descriptor, associator, matcher)(inputA, inputA) // for overlap

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

    // Convert the output image to a BufferedImage.
    val stitched =
      new BufferedImage(colorA.width, colorA.height, imageA.getType)
    ConvertBufferedImage.convertTo(colorA, stitched, true)
  }

}
