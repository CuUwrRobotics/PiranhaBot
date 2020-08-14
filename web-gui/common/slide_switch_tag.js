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
  var createLabel, id, name, disable;
  if (!element.attributes.var) {
    console.error("No id given for x-slideswitch tag.");
    return;
  }
  createLabel = true;
  if (!element.attributes.text) {
    createLabel = false;
  }
  id = element.attributes.var.value;
  name = element.attributes.text.value;
  if (element.attributes.disabled) disable = true;
  else disable = false;

  element.innerHTML = "";
  if (createLabel) {
    element.innerHTML += "<label class=\"switch-label\" for=\"" + id + "\">" +
      name + " </label>";
  }
  element.innerHTML += "<label class=\"switch\"><input " +
    ((disable) ? "disabled" : "") +
    " type=\"checkbox\" id=\"" + id +
    "\">\n<span class=\"slider round\"></span></label>";
}

customTag("x-slideswitch", createSlideSwitch);
