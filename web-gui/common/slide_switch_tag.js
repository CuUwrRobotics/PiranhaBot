// This JS file will allow for a custom HTML tage to create a slide switch with only one line.

function customTag(tagName, fn) {
  document.createElement(tagName);
  // find all the tags occurrences (instances) in the document
  var tagInstances = document.getElementsByTagName(tagName);
  // for each occurrence run the associated function
  for (var i = 0; i < tagInstances.length; i++) {
    fn(tagInstances[i]);
  }
}

function createSlideSwitch(element) {
  // code for rendering the element goes here
  if (!element.attributes.val) {
    console.error("No id given for x-slideswitch tag.");
    return;
  }
  if (!element.attributes.text) {
    console.error("No text given for x-slideswitch tag.");
    return;
  }
  var id = element.attributes.val.value;
  var name = element.attributes.text.value;
  element.innerHTML = "<label class=\"switch-label\" for=\"" + id + "\">" + name + " </label>";
  // element.innerHTML += ;
  element.innerHTML += "<label class=\"switch\"><input type=\"checkbox\" id=\"" + id + "\">\n<span class=\"slider round\"></span></label>";

}

customTag("x-slideswitch", createSlideSwitch);
