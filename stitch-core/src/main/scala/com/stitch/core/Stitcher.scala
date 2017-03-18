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
import boofcv.io.image.ConvertBufferedImage
import boofcv.struct.feature.BrightFeature
import boofcv.struct.geo.AssociatedPair
import boofcv.struct.image.GrayF32
import georegression.struct.homography.Homography2D_F64
import georegression.struct.point.Point2D_F64
import java.awt.image.BufferedImage
import org.ddogleg.fitting.modelset.ModelMatcher
import org.ddogleg.struct.FastQueue
import scala.collection.mutable
import scala.collection.JavaConverters._
import java.awt.Dimension
import java.io.File
import javax.imageio.ImageIO

object Stitcher {

  def main(args: Array[String]): Unit = {

    // Input parameters.
    val size   = new Dimension(1920, 1080)
    val video  = "stitch-assets/balcony.mp4"
//    val dir    = "/Users/ashwin/Downloads/stitcher" // Ashwin's computer
    val dir = "stitch-assets/video-frames" // Eric's setup
    val output = "stitch-assets/balcony-stitched.mp4"
    val look   = 300
    val jump   = 25

//    // Extract frames from the video using ffmpeg.
//    ffmpeg(video, dir + "/frame-%07d.png")
//    println("done splitting video into individual frames")

    val frameFiles = new File(dir).listFiles(f => f.getName.matches("frame-[0-9]+.png")).sortWith(_.getName < _.getName)

    val numFrames = frameFiles.length
    val frameSpace = frameFiles.length
    val capacity = 200 // TODO: use Runtime to determine how many images to read in at a time

    var homographies = Seq[Homography2D_F64]()
    var nextFrame = 0
    while (nextFrame < numFrames) {
      println(nextFrame)
      val frames = frameFiles
        .slice(nextFrame, (nextFrame + capacity) min (numFrames - 1))
        .map(ImageIO.read)
        .toSeq

      homographies = homographies ++ incrementalTransform(frames) // what is the best way to concat sequences? use another data structure?
      nextFrame += capacity - 1
    }

//    // Load all the extracted frames. TODO: what if the video is too large for memory?
//    val frames = new File(dir)
//      .listFiles(f => f.getName.matches("frame-[0-9]+.png"))
//      .sortWith(_.getName < _.getName)
//      .map(ImageIO.read)
//      .toSeq
//
//    println("done loading frames")
//
//    // Stitch together the frames and write to file in parallel.
//    val stitched = frames.zipWithIndex.par.foreach { case (f, i) => // readd par before foreach
//      val before = ((i - jump) to (0 max (i - look)) by -jump).map(frames).reverse
//      val after  = ((i + jump) to (frames.length-1 min (i + look)) by jump).map(frames)
//      ImageIO.write(stitch(size)(f, before, after), "png", new File(dir + "/stitched-%07d.png".format(i)))
//      println(i)
//    }
//
//    println("done generating stitched frames")
//
//    // Generate a video from the stitched images.
//    ffmpeg(dir + "/stitched-%07d.png", output)
//    new File(dir).delete() // comment out to inspect stitched frames
  }

  def play(): Unit = {
    // Input parameters.
    val size   = new Dimension(1920, 1080)
    val video  = "stitch-assets/balcony.mp4"
    //    val dir    = "/Users/ashwin/Downloads/stitcher" // Ashwin's computer
    val dir = "stitch-assets/video-frames" // Eric's setup
    val output = "stitch-assets/balcony-stitched.mp4"
    val look   = 22
    val jump   = 20

    val originalIndex = 220
    val stitchIndex = 200

    // This does not maintain (our) ordering of the frames we generated (BAD)
    val frames = new File(dir)
      .listFiles(f => f.getName.matches("frame-[0-9]+.png"))
      .sortWith(_.getName < _.getName)
      .map(ImageIO.read)
      .toSeq

    ImageIO.write(frames(200), "png", new File("stitch.png"))
    ImageIO.write(frames(220), "png", new File("original.png"))

    val before = frames(stitchIndex) :: Nil
    var working = new BufferedImage(size.width * 2, size.height * 2, frames(originalIndex).getType)
    draw(working, frames(originalIndex), size)

    // Setup the rendering toolchain.
    val model = new PixelTransformHomography_F32
    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    val backward = transform(working, before)

    val color = ConvertBufferedImage.convertFromMulti(working, null, true, classOf[GrayF32])
    val frame = ConvertBufferedImage.convertFromMulti(frames(stitchIndex), null, true, classOf[GrayF32])

    model.set(backward(0))
    distortion.apply(frame, color)

    working = ConvertBufferedImage.convertTo(color, new BufferedImage(color.width, color.height, working.getType), true)

    draw(working, frames(originalIndex), size)

    ImageIO.write(working, "png", new File("output.png"))
  }

  /**
   * Calls ffmpeg to either (1) extract frames from a video or (2) generate a video from frames.
   * Each function is called by swapping the input and output parameters.
   *
   * @param input Input data.
   * @param output Output location.
   */
  def ffmpeg(input: String, output: String): Unit = {
    new ProcessBuilder("mkdir", "-p", output).start().waitFor() // TODO: huh? (kind of messy I guess)
    new ProcessBuilder("rm", "-r", output).start().waitFor()
    val pb = new ProcessBuilder("ffmpeg", "-i", input, output)
    pb.start().waitFor() // TODO: check for error code (make folder first)
  }

  /**
   * Returns an image of the provided dimensions that is generated by stitched the provided before
   * and after frames to the original frame. Before frames are specified in ascending order of time
   * and after frames are specified in descending order of time; therefore, the first before frame
   * is the oldest and the last after frame is the latest.
   *
   * @param size Dimensions of stitched image.
   * @param original Original frame.
   * @param before Frames before the original.
   * @param after Frames after the original.
   * @return Stitched image of the provided dimensions.
   */
  def stitch(
    size: Dimension
  )(
    original: BufferedImage,
    before: Seq[BufferedImage],
    after: Seq[BufferedImage]
  ): BufferedImage = {
    // Construct the working image. We use a significantly larger working image than the desired
    // dimensions of the output image, because otherwise there aren't enough descriptors to perform
    // matching for frames near the extreme edges.
    var working = new BufferedImage(size.width * 2, size.height * 2, original.getType)
    draw(working, original, size)

    // Setup the rendering toolchain.
    val model = new PixelTransformHomography_F32
    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    // Calculate the transformations between each frames and the original.
    val backward = transform(working, before)
    val forward = transform(working, after.reverse).reverse

    // Zip together the backwards and forwards transformations to produce a sequence of
    // transformations that correspond to an offset of -1, 1, -2, 2, etc. from the original frame.
    val transformations = before.zip(backward).map(Seq(_))
      .zipAll(after.zip(forward).map(Seq(_)), Seq.empty, Seq.empty)
      .flatMap { case (a, b) => a ++ b }
      .reverse

    // Apply the transformations in reverse time-order to ensure that the nearest frames to the
    // original are applied first. Render each transformation to the underlying working copy.
    transformations.reverse.foreach { case (f, h) =>
      val color = ConvertBufferedImage.convertFromMulti(working, null, true, classOf[GrayF32])
      val frame = ConvertBufferedImage.convertFromMulti(f, null, true, classOf[GrayF32])

      model.set(h)
      distortion.apply(frame, color)
      working = ConvertBufferedImage.convertTo(color, new BufferedImage(color.width, color.height, working.getType), true)
    }

    // Draw the original frame back onto the working image.
    draw(working, original, size)

    // Extract the sub-image of the desired dimension from the working image.
    val dx = (working.getWidth - size.width) / 2
    val dy = (working.getHeight - size.height) / 2
    working.getSubimage(dx, dy, size.width, size.height)
  }

  /**
   * Scales the image to the provided dimensions, while preserving aspect ratio, and then draws the
   * resized image in the center of the provided canvas.
   *
   * @param canvas Image to draw on.
   * @param image Image to draw.
   * @param size Scaled dimensions of image.
   */
  def draw(
    canvas: BufferedImage,
    image: BufferedImage,
    size: Dimension
  ): Unit = {
    // Calculate the width, height, and aspect ratio of the image.
    val w = image.getWidth
    val h = image.getHeight
    val r = w / h.toDouble
    val graphics = canvas.getGraphics

    // Resize the image while preserving aspect ratio.
    if (h > w) {
      val width = (size.getHeight * r).toInt
      val dx = (canvas.getWidth - width) / 2
      val dy = (canvas.getHeight - size.height) / 2
      graphics.drawImage(image, dx, dy, dx + width, dy + size.height, 0, 0, w, h, null)
    } else {
      val height = (size.getWidth / r).toInt
      val dx = (canvas.getWidth - size.width) / 2
      val dy = (canvas.getHeight - height) / 2
      graphics.drawImage(image, dx, dy, dx + size.width, dy + size.height, 0, 0, w, h, null)
    }

    // Cleanup the graphics instance.
    graphics.dispose()
  }

  /**
   * Returns a sequence of homographies that map each preceding frame to the current frame.
   * Preceding frames are specified in ascending order of time, culminating in the current frame;
   * therefore, the first frame in the sequence is the oldest and the last is the latest.
   * Homographies are first calculated between adjacent frames, and then are concatenated together.
   *
   * @param current Current frame.
   * @param preceding Preceding frames.
   * @return Sequence of homographies that transform preceding frames to the current.
   */
  def transform(
    current: BufferedImage,
    preceding: Seq[BufferedImage]
  ): Seq[Homography2D_F64] = {
    // Setup the transformation toolchain.
    val descriptor = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null, classOf[GrayF32])
    val associator = FactoryAssociation.greedy(FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true), 2, true)
    val matcher = FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))

    // Calculate the homographies between adjacent frames.
    val homographies = (preceding :+ current)
      .map(ConvertBufferedImage.convertFromSingle(_, null, classOf[GrayF32]))
      .sliding(2)
      .map(x => homography(descriptor, associator, matcher)(x.last, x.head))
      .toSeq

    // Concatenate homographies to find absolute transformations between frames and the original.
    val id = new Homography2D_F64(1, 0, 0, 0, 1, 0, 0, 0, 1)
    homographies.scanRight(id)((l, r) => r.concat(l, null)).take(preceding.length)
  }

  /**
    * Takes in a sequence of consecutive frames and compute the homographies between each frame t and t+1.
    * @param frames Frames from video.
    * @return Sequence of homographies that transform frame t to t-1.
    */
  def incrementalTransform(
    frames: Seq[BufferedImage]
  ): Seq[Homography2D_F64] = {
    val descriptor = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null, classOf[GrayF32])
    val associator = FactoryAssociation.greedy(FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true), 2, true)
    val matcher = FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))

    frames
      .map(ConvertBufferedImage.convertFromSingle(_, null, classOf[GrayF32]))
      .sliding(2)
      .map(x => homography(descriptor, associator, matcher)(x.last, x.head))
      .toSeq
  }

  /**
   * Returns a homography from image B to A. Utilizes the provided detection algorithm to find
   * interest points, the provided association algorithm to group them, and the provided matching
   * model to determine the homography.
   *
   * @param descriptor Detection algorithm.
   * @param associator Association algorithm.
   * @param matcher Matching algorithm.
   * @param inputA First input image.
   * @param inputB Second input image.
   * @return Homography from image B to image A.
   */
  def homography(
    descriptor: DetectDescribePoint[GrayF32, BrightFeature],
    associator: AssociateDescription[BrightFeature],
    matcher: ModelMatcher[Homography2D_F64, AssociatedPair]
  )(
    inputA: GrayF32,
    inputB: GrayF32
  ): Homography2D_F64 = {
    // Locate a matching between the interest points of both images.
    val (pointsA, descA) = describe(descriptor)(inputA)
    val (pointsB, descB) = describe(descriptor)(inputB)

    associator.setSource(descA)
    associator.setDestination(descB)
    associator.associate()
    val matches = associator.getMatches

    // Construct an association between points.
    val pairs = mutable.Buffer.empty[AssociatedPair]
    (0 until matches.size).map(matches.get).foreach { m =>
      pairs += new AssociatedPair(pointsA(m.src), pointsB(m.dst), false)
    }

    // Attempt to construct the homography.
    if(!matcher.process(pairs.asJava))
      throw new RuntimeException("Unable to determine homography.")
    else
      matcher.getModelParameters.copy()
  }

  /**
   * Returns a tuple containing the location and descriptions of the various interest points in the
   * provided image. Interest points are found using the provided detection algorithm. Location and
   * description indices correspond.
   *
   * @param detector Detection algorithm.
   * @param input Input image.
   * @return Location and description of interest points.
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

}