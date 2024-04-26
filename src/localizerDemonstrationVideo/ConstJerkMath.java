package localizerDemonstrationVideo;

import localizer.ConstJerkLocalizer;
import localizer.Localizer;

public class ConstJerkMath {
	public static void main(String[] args) {
		Localizer l = new ConstJerkLocalizer();
		LocalizerDemostrationVideo vid = new LocalizerDemostrationVideo(l);
	}
}