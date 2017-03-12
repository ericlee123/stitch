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
    new ProcessBuilder("mkdir", frameFolder).start().waitFor()
    val pb = new ProcessBuilder("ffmpeg", "-i", videoPath, frameFolder + "/image-%07d.png")
    val p = pb.start().waitFor()
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
