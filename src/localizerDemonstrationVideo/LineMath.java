package localizerDemonstrationVideo;

import localizer.LinearLocalizer;
import localizer.Localizer;

public class LineMath {
	public static void main(String[] args) {
		Localizer l = new LinearLocalizer();
		LocalizerDemostrationVideo vid = new LocalizerDemostrationVideo(l);
	}
}
