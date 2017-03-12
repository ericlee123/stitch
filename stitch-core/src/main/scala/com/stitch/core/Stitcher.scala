// format: OFF

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

  val videoPath = "stitch-assets/axis-allies.mp4"

  def main(args: Array[String]): Unit = {

    // generate individual frames on disk
    // stitch them together given video dimensions

    val width = 1920
    val height = 1080

    val videoPath = "stitch-assets/balcony.mp4"
    val frameFolder = generateVideoFrames(videoPath)

    generateStitchedImages(frameFolder, width, height)
  }

  def generateVideoFrames( videoPath: String ): String = {
    //generate frames using ffmpeg (because build and other tools are fucked)
    print("generating video frames...")
    val frameFolder = "video-frames"
//    new ProcessBuilder("mkdir", frameFolder).start().waitFor()
//    val pb = new ProcessBuilder("ffmpeg", "-i", videoPath, frameFolder + "/image-%07d.png")
//    val p = pb.start().waitForh b zr e()
    println(" done")
    frameFolder
  }

  def generateStitchedImages( frameFolder: String, width: Int, height: Int): Unit = {

    val outputFolder = "stitched-video-frames"
    new ProcessBuilder("mkdir", outputFolder).start().waitFor()

    // figure out how many frames were generated
    val numFrames = new File(frameFolder).listFiles().length
    val lookDistance = 150
    val jumpDistance = 25
    val spicyCurry = stitchFrame(width, height, lookDistance, jumpDistance, frameFolder, numFrames)(_)
    for (i <- 1 to numFrames) { // TODO: probably should do some smart functional mapping stuff here
      println("stitching frame " + i + "...")
      ImageIO.write(spicyCurry(i), "png", new File(outputFolder + "/image-%07d.png".format(i)))
//      println(" done")
    }
  }

  def stitchFrame( width: Int, height: Int, lookDistance: Int, jumpDistance: Int, frameFolder: String, numFrames: Int )( frameIndex: Int ): BufferedImage = {
    val nakedFrame = ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex)))
    var blackBarred = getSizedFrame(nakedFrame, width, height)

    // make the screen wider than necessary for extra descriptors
    val wideSize = 4000
    var wideFrame = new BufferedImage(wideSize, wideSize, blackBarred.getType())
    wideFrame.createGraphics().drawImage(blackBarred, (wideSize - height) / 2, (wideSize - width) / 2, width, height, null)

    val backward = ListBuffer[Homography2D_F64]()
    val forward = ListBuffer[Homography2D_F64]()

    // Setup the interest point descriptor, associator, and matcher.
    val descriptor = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null, classOf[GrayF32])
    val associator = FactoryAssociation.greedy(FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true), 2, true)
    val matcher = FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))
    val greenCurry = computeHomography(descriptor, associator, matcher)( _,_ )

    for (i <- jumpDistance to lookDistance by jumpDistance) {
      if (frameIndex - i > 0) {
        val further = ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex - i)))
        var closer = ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex - i + jumpDistance))) // this code is ugly
        if (i == jumpDistance) {
          closer = wideFrame
        }
        backward += greenCurry(closer, further)
      }
      if (frameIndex + i <= numFrames) {
        val further = ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex + i)))
        var closer = ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex + i - jumpDistance))) // this is also ugly
        if (i == jumpDistance) {
          closer = wideFrame
        }
        forward += greenCurry(closer, further)
      }
    }

    // Setup the rendering toolchain.
    val model = new PixelTransformHomography_F32
    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    for (i <- backward.length - 1 to 0 by -1) {
      var homo = backward(0)
      for (j <- 1 to i) {
        homo = homo.concat(backward(j), null)
      }
//      print(" " + (frameIndex - (i * jumpDistance)))
      val colorMain = ConvertBufferedImage.convertFromMulti(wideFrame, null, true, classOf[GrayF32])
      val colorCurrent = ConvertBufferedImage.convertFromMulti(ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex - ((i+1) * jumpDistance)))), null, true, classOf[GrayF32])
      model.set(homo)
      distortion.apply(colorCurrent, colorMain)

      wideFrame = ConvertBufferedImage.convertTo(colorMain, new BufferedImage(colorMain.width, colorMain.height, wideFrame.getType()), true)
    }

    for (i <- forward.length - 1 to 0 by -1) {
      var homo = forward(0)
      for (j <- 1 to i) {
        homo = homo.concat(forward(j), null)
      }
//      print(" " + (frameIndex + (i * jumpDistance)))
      val colorMain = ConvertBufferedImage.convertFromMulti(wideFrame, null, true, classOf[GrayF32])
      val colorCurrent = ConvertBufferedImage.convertFromMulti(ImageIO.read(new File(frameFolder + "/image-%07d.png".format(frameIndex + ((i+1) * jumpDistance)))), null, true, classOf[GrayF32])
      model.set(homo)
      distortion.apply(colorCurrent, colorMain)

      wideFrame = ConvertBufferedImage.convertTo(colorMain, new BufferedImage(colorMain.width, colorMain.height, wideFrame.getType()), true)
    }
    val originalFrame = ImageIO.read(new File(frameFolder + "/image-%07.png"))
    wideFrame.createGraphics()
      .drawImage(originalFrame, (wideSize - height) / 2, (wideSize - width) / 2, width, height, null) // paste the original image on top
    wideFrame.getSubimage((wideSize - height) / 2, (wideSize - width) / 2, width, height)

//    val twoBack = ImageIO.read(new File("video-frames/image-0000180.png"))
//    val oneBack = ImageIO.read(new File("video-frames/image-0000200.png"))
//
//    // do the homography
//    val inputMain = ConvertBufferedImage.convertFromSingle(wideFrame, null, classOf[GrayF32])
//    val inputA = ConvertBufferedImage.convertFromSingle(twoBack, null, classOf[GrayF32])
//    val inputB = ConvertBufferedImage.convertFromSingle(oneBack, null, classOf[GrayF32])
//
//    val colorMain = ConvertBufferedImage.convertFromMulti(wideFrame, null, true, classOf[GrayF32])
//    val colorA = ConvertBufferedImage.convertFromMulti(twoBack, null, true, classOf[GrayF32])
//    val colorB = ConvertBufferedImage.convertFromMulti(oneBack, null, true, classOf[GrayF32])
//
//    val b2a = fuck(oneBack, twoBack)
//    val main2b = fuck(wideFrame, oneBack)
//
//    // Setup the rendering toolchain.
//    val model = new PixelTransformHomography_F32
//    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
//    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
//    distortion.setRenderAll(false)
//
//    val main2a = main2b.concat(b2a, null)
//    model.set(main2b)
//    distortion.apply(colorB, colorMain)
//    model.set(main2a)
//    distortion.apply(colorA, colorMain)
//
//    val stitched = new BufferedImage(colorMain.width, colorMain.height, wideFrame.getType())
//    ConvertBufferedImage.convertTo(colorMain, stitched, true)

//    doHomography( wideFrame, frameFolder )( 180 )

//    val redCurry = doHomography(blackBarred) // CAN I DO THIS???
//    for (i <- jumpDistance to lookDistance by jumpDistance) { // TODO: do some cool functional mapping here too
//      if (frameIndex + i <= numFrames) {
//        wideFrame = doHomography(wideFrame, frameFolder)(frameIndex + i)
//      }
////      if (frameIndex - i > 0) {
////        wideFrame = doHomography(wideFrame, frameFolder)(frameIndex - i)
////      }
//    }
//    wideFrame
  }

  def getSizedFrame( frame: BufferedImage, width: Int, height: Int): BufferedImage = {
    // TODO: don't assume that the black bars are on the side (safe assumption though)
    var scaledWidth = Math.floor((height.toDouble / frame.getHeight()) * frame.getWidth()).toInt

    var resized = new BufferedImage(scaledWidth, height, BufferedImage.TYPE_INT_RGB)
    resized.createGraphics().drawImage(frame, 0, 0, scaledWidth, height, null)

    var wideFrame = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB)
    val offset = (width - resized.getWidth()).toDouble / 2
    wideFrame.createGraphics().drawImage(resized, offset.toInt, 0, resized.getWidth(), resized.getHeight(), null)
    wideFrame
  }

  def doHomography( currentFrame: BufferedImage, frameFolder: String )( stitchIndex: Int ): BufferedImage = {
    // TODO: tweak this later to be better
    // Setup the interest point descriptor, associator, and matcher.
    val descriptor = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null, classOf[GrayF32])
    val associator = FactoryAssociation.greedy(FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true), 2, true)
    val matcher = FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))

    // Setup the rendering toolchain.
    val model = new PixelTransformHomography_F32
    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    val otherImage = ImageIO.read(new File(frameFolder + "/image-%07d.png".format(stitchIndex + 1)))

    // do the homography
    val inputMain = ConvertBufferedImage.convertFromSingle(currentFrame, null, classOf[GrayF32])
    val inputOther = ConvertBufferedImage.convertFromSingle(otherImage, null, classOf[GrayF32])

    val colorMain = ConvertBufferedImage.convertFromMulti(currentFrame, null, true, classOf[GrayF32])
    val colorOther = ConvertBufferedImage.convertFromMulti(otherImage, null, true, classOf[GrayF32])

    val other2Main = homography(descriptor, associator, matcher)(inputMain, inputOther)

    model.set(other2Main)
    distortion.apply(colorOther, colorMain)

    val stitched = new BufferedImage(colorMain.width, colorMain.height, currentFrame.getType())
    ConvertBufferedImage.convertTo(colorMain, stitched, true)
  }

  def computeHomography( descriptor: DetectDescribePoint[GrayF32, BrightFeature], associator: AssociateDescription[BrightFeature], matcher: ModelMatcher[Homography2D_F64, AssociatedPair] )
                       ( mainFrame: BufferedImage, otherFrame: BufferedImage ): Homography2D_F64 = {
    // Setup the rendering toolchain.
    val model = new PixelTransformHomography_F32
    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    // compute
    val inputMain = ConvertBufferedImage.convertFromSingle(mainFrame, null, classOf[GrayF32])
    val inputOther = ConvertBufferedImage.convertFromSingle(otherFrame, null, classOf[GrayF32])

    val colorMain = ConvertBufferedImage.convertFromMulti(mainFrame, null, true, classOf[GrayF32])
    val colorOther = ConvertBufferedImage.convertFromMulti(otherFrame, null, true, classOf[GrayF32])

    homography(descriptor, associator, matcher)(inputMain, inputOther)
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

}
